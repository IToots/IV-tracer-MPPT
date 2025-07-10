# ----------- IMPORTS -----------
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import sys
import time
import serial
import numpy as np
import pandas as pd
import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

# ----------- LOAD REFERENCE DATA -----------
# Choose reference file for solar cell comparison
FILE_ref = "1V_02W.txt"

# Load and clean reference data
df_ref = pd.read_csv(FILE_ref, skiprows=9, delim_whitespace=True)
df_ref = df_ref.dropna(subset=['Vmeas', 'Imeas'])  # Remove NaNs in key columns
df_ref['Vmeas'] = pd.to_numeric(df_ref['Vmeas'], errors='coerce')
df_ref['Imeas'] = pd.to_numeric(df_ref['Imeas'], errors='coerce')
df_ref = df_ref.dropna()
# Extract voltage and current arrays
voltage_ref = df_ref['Vmeas']
current_ref = df_ref['Imeas']
max_voltage_ref = voltage_ref.max()
max_current_ref = current_ref.max()

# ----------- PLOT WIDGET CLASS -----------
class PltFigure(FigureCanvasQTAgg):
    """Custom Matplotlib Figure class embedded in PyQt"""
    def __init__(self, parent, xlabel, ylabel):
        self.fig = Figure()
        super().__init__(self.fig)
        self.setParent(parent)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)
        self.axes.plot(voltage_ref, current_ref, 'k--', label="Reference")
        self.axes.set_xlim(0, max_voltage_ref * 1.1)
        self.axes.set_ylim(0, max_current_ref * 1.1)
        self.axes.legend()
        self.draw()

    def reset_plot(self):
        """Reset the plot to show only the reference curve"""
        self.axes.clear()
        self.axes.set_xlabel("Voltage (V)")
        self.axes.set_ylabel("Current (A)")
        self.axes.plot(voltage_ref, current_ref, 'k--', label="Reference")
        self.axes.set_xlim(0, max_voltage_ref * 1.1)
        self.axes.set_ylim(0, max_current_ref * 1.1)
        self.axes.legend()
        self.draw()

# ----------- SERIAL COMMUNICATION THREAD -----------
class Connect(QThread):
    """Worker thread to communicate with Raspberry Pi Pico over serial"""
    update_data = pyqtSignal(str)
    finished_collecting = pyqtSignal()
    def __init__(self, command):
        super().__init__()
        self.command = command  # Serial command to send (e.g., "START\n")
    def run(self):
        """Main logic to send command and read data lines until END or FAILED"""
        print(f"Starting worker thread with command: {self.command}")
        with serial.Serial('COM14', 115200, timeout=1) as conn:
            conn.write(self.command.encode())  # Send command to Pico
            while True:
                if conn.in_waiting > 0:
                    line = conn.readline().decode(errors="ignore").strip()
                    if line == "END":
                        self.finished_collecting.emit()
                        break
                    elif line == "FAILED":
                        print("FAILED")
                        break
                    self.update_data.emit(line)
        time.sleep(0.5)
        print("Finished Thread")

# ----------- MAIN GUI CLASS -----------
class GUI(QtWidgets.QMainWindow):
    """Main application window"""
    def __init__(self):
        super().__init__()
        uic.loadUi("MainWindow.ui", self)
        # State variables
        self.previous_frequency = None
        self.current_frequency = None
        self.freq_data = {}          # Stores voltage/current data per frequency
        self.color_map = {}          # Maps each frequency to a color
        self.colors = matplotlib.colormaps['tab10'].colors  # Color cycle
        self._voltage = []
        self._current = []
        self.results = []
        # Set up embedded Matplotlib plot
        self.plot_widget = PltFigure(self.Graph, "Voltage (V)", "Current (A)")
        layout = QtWidgets.QVBoxLayout(self.Graph)
        layout.addWidget(self.plot_widget)
        # Button connections
        self.Sweep.clicked.connect(self.send_sweep_command)
        self.Single.clicked.connect(self.send_single_command)
        self.Reset.clicked.connect(self.reset)
        # Display reference parameters on startup
        self.parameters(voltage_ref.values, current_ref.values)
        self.show()

    # ----------- BUTTON HANDLERS -----------
    def send_sweep_command(self):
        """Start full sweep from Pico"""
        self._voltage.clear()
        self._current.clear()
        self.worker = Connect("SWEEP\n")
        self.worker.update_data.connect(self.update_plot)
        self.worker.finished_collecting.connect(self.on_data_finished)
        self.worker.start()

    def send_single_command(self):
        """Request a single I-V point from Pico"""
        self._voltage.clear()
        self._current.clear()
        self.worker = Connect("SINGLE\n")
        self.worker.update_data.connect(self.update_plot)
        self.worker.finished_collecting.connect(self.on_data_finished)
        self.worker.start()

    # ----------- DATA HANDLER -----------
    @pyqtSlot(str)
    def update_plot(self, data: str):
        """Handle incoming serial data line"""
        print(data)
        # Frequency line: start of a new sweep
        if data.startswith("FREQ"):
            try:
                _, freq = data.strip().split(",")
                freq = float(freq)

                # Finalize previous frequency's data
                if self.previous_frequency is not None:
                    prev_data = self.freq_data[self.previous_frequency]
                    if 0.0 not in prev_data['voltage']:
                        isc = max(prev_data['current']) if prev_data['current'] else 0.0
                        prev_data['voltage'].append(0.0)
                        prev_data['current'].append(isc)
                    self.parameters(np.array(prev_data['voltage']),
                                    np.array(prev_data['current']),
                                    freq=self.previous_frequency)
                    # Save CSV
                    df = pd.DataFrame(prev_data)
                    filename = f"sweep_{int(self.previous_frequency)}Hz.csv"
                    df.to_csv(filename, index=False)
                    print(f"Saved sweep to {filename}")

                self.current_frequency = freq
                self.previous_frequency = freq
                # Allocate data list and color
                if self.current_frequency not in self.freq_data:
                    self.freq_data[self.current_frequency] = {'voltage': [], 'current': []}
                    color_idx = len(self.color_map) % len(self.colors)
                    self.color_map[self.current_frequency] = self.colors[color_idx]
                print(f"Current Frequency: {self.current_frequency} Hz")
            except Exception as e:
                print(f"Invalid data format: {data} -> {e}")
            return
        # Data line (time, duty, V, V_c, I, P, P_c)
        try:
            _, duty, voltage,voltage_corrected, current, power, power_corrected = map(float, data.strip().split(","))
            if self.current_frequency is not None:
                v_list = self.freq_data[self.current_frequency]['voltage']
                i_list = self.freq_data[self.current_frequency]['current']
                # Avoid duplicate points
                if voltage_corrected in v_list or current in i_list:
                    return
                v_list.append(voltage_corrected)
                i_list.append(current)
                self.update_combined_plot()
        except Exception as e:
            print(f"Invalid data format: {e}")

    def update_combined_plot(self):
        """Redraw plot with all frequencies + reference"""
        self.plot_widget.axes.clear()
        self.plot_widget.axes.set_xlabel("Voltage (V)")
        self.plot_widget.axes.set_ylabel("Current (A)")
        max_voltage = max(voltage_ref) if len(voltage_ref) > 0 else 0
        max_current = max(current_ref) if len(current_ref) > 0 else 0
        # Reference
        self.plot_widget.axes.plot(voltage_ref, current_ref, 'k--', label="Reference")
        # All frequency sweeps
        for freq, data in self.freq_data.items():
            color = self.color_map.get(freq, 'k')
            label = f"{freq/1000:.0f} kHz"
            self.plot_widget.axes.plot(data['voltage'], data['current'], color=color, label=label)
            if data['voltage']:
                max_voltage = max(max_voltage, max(data['voltage']))
            if data['current']:
                max_current = max(max_current, max(data['current']))
        self.plot_widget.axes.set_xlim(0, max_voltage * 1.1)
        self.plot_widget.axes.set_ylim(0, max_current * 1.1)
        self.plot_widget.axes.legend()
        self.plot_widget.draw()

    # ----------- RESET -----------
    def reset(self):
        """Reset the whole interface"""
        self.plot_widget.reset_plot()
        self.freq_data.clear()
        self.color_map.clear()
        self._voltage.clear()
        self._current.clear()
        self.current_frequency = None
        self.previous_frequency = None
        self.results.clear()
        self.ResultsTable.clearContents()
        self.ResultsTable.setRowCount(0)

    # ----------- PARAMETER CALCULATION -----------
    def parameters(self, voltage, current, freq="N/A"):
        """Calculate and display Voc, Isc, MPP, Rs, Rsh"""
        # Open-circuit voltage (Voc) = V where I ≈ 0
        voc = voltage[np.argmin(np.abs(current))]
        # Short-circuit current (Isc) = I where V ≈ 0
        isc = current[np.argmin(np.abs(voltage))]
        # Max power point (Pmax)
        power = voltage * current
        pmax_idx = np.argmax(power)
        vmp, imp, pmax = voltage[pmax_idx], current[pmax_idx], power[pmax_idx]
        # Series resistance (Rs): negative slope near Isc
        rs = np.nan
        try:
            best_rs = None
            for n in range(3, min(20, len(voltage) + 1)):
                coeffs = np.polyfit(voltage[:n], current[:n], 1)
                candidate_rs = -1 / coeffs[0]
                if best_rs is None or (0 < candidate_rs < best_rs):
                    best_rs = candidate_rs
            rs = best_rs
        except Exception as e:
            print(f"Error calculating Rs: {e}")
        # Shunt resistance (Rsh): negative slope near Voc
        rsh = np.nan
        try:
            best_rsh = None
            for n in range(3, min(20, len(voltage) + 1)):
                coeffs = np.polyfit(voltage[-n:], current[-n:], 1)
                candidate_rsh = -1 / coeffs[0]
                if best_rsh is None or candidate_rsh > best_rsh:
                    best_rsh = candidate_rsh
            rsh = best_rsh
        except Exception as e:
            print(f"Error calculating Rsh: {e}")
        # Store and display in table
        self.results.append({
            'Frequency (Hz)': freq,
            'Voc (V)': round(voc, 3),
            'Isc (A)': round(isc, 3),
            'Vmp (V)': round(vmp, 3),
            'Imp (A)': round(imp, 3),
            'Pmax (W)': round(pmax, 3),
            'Rs (?)': round(rs, 2) if rs is not None else 'N/A',
            'Rsh (?)': round(rsh, 2) if rsh is not None else 'N/A',
        })
        # Update table
        headers = list(self.results[0].keys())
        self.ResultsTable.setColumnCount(len(headers))
        self.ResultsTable.setHorizontalHeaderLabels(headers)
        self.ResultsTable.setRowCount(len(self.results))
        for row_idx, data_dict in enumerate(self.results):
            for col_idx, key in enumerate(headers):
                item = QTableWidgetItem(str(data_dict.get(key, "")))
                self.ResultsTable.setItem(row_idx, col_idx, item)
        self.ResultsTable.resizeColumnsToContents()
    
    # ----------- END OF DATA -----------
    @pyqtSlot()
    def on_data_finished(self):
        """Handle completion of sweep"""
        if self.current_frequency is not None:
            v_arr = np.array(self.freq_data[self.current_frequency]['voltage'])
            i_arr = np.array(self.freq_data[self.current_frequency]['current'])
            # Add zero-voltage point if not already present
            if 0.0 not in self.freq_data[self.current_frequency]['voltage']:
                self.freq_data[self.current_frequency]['voltage'].append(0.0)
                self.freq_data[self.current_frequency]['current'].append(np.max(i_arr))
            self.update_combined_plot()
            self.parameters(v_arr, i_arr, freq=self.current_frequency)
            # Save final data to CSV
            df = pd.DataFrame({
                'Voltage (V)': v_arr,
                'Current (A)': i_arr
            })
            filename = f"sweep_{int(self.current_frequency)}Hz.csv"
            df.to_csv(filename, index=False)
            print(f"Saved sweep to {filename}")

# ----------- MAIN LAUNCHER -----------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
