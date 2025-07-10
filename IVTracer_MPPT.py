# ----------- IMPORTS -----------
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import sys
import time
import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
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
        self.axes.set_xlim(left=0)
        self.axes.set_ylim(bottom=0)
        self.draw()

    def reset_plot(self):
        """Reset the plot to empty (no data)"""
        self.axes.clear()
        self.axes.set_xlabel("")
        self.axes.set_ylabel("")
        self.axes.set_xlim(left=0)
        self.axes.set_ylim(bottom=0)
        self.draw()

# ----------- SERIAL COMMUNICATION THREAD -----------
class Connect(QThread):
    """Worker thread to communicate with Raspberry Pi Pico over serial"""
    update_data = pyqtSignal(str)
    finished_collecting = pyqtSignal()
    def __init__(self, command):
        super().__init__()
        self.command = command
        self._send_command = None
        self._running = True

    def send_command(self, command):
        """Send command to Pico using existing connection"""
        if self._send_command is None:
            self._send_command = command

    def run(self):
        print(f"Starting worker thread with command: {self.command.strip()}")
        try:
            with serial.Serial('COM14', 115200, timeout=1) as conn:
                conn.write(self.command.encode())  # Send command to Pico
                while True:
                    if self._send_command == "STOP":
                        print("Stop Sent")
                        conn.write("STOP\n")
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
        except Exception as e:
            print(f"Serial connection error: {e}")
        print("Finished Thread")

        def stop(self):
            self._running = False

        def __repr__(self):
            return f"{self.command}"

# ----------- MAIN GUI CLASS -----------
class GUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("MainWindow.ui", self)
        
        # State variables
        self.previous_frequency = None
        self.current_frequency = None
        self.freq_data = {}
        self.plot_mode = "IV"  # or "MPPT"
        self._voltage = []
        self._voltage_corrected = []
        self._current = []
        self.results = []
        self._time = []
        self._power = []
        self._power_corrected = []
        self._worker = None
        
        # Set up embedded Matplotlib plot (starts empty)
        self.plot_widget = PltFigure(self.Graph, "", "")
        layout = QtWidgets.QVBoxLayout(self.Graph)
        layout.addWidget(self.plot_widget)
        
        # Connect buttons
        self.Sweep.clicked.connect(self.send_sweep_command)
        self.Single.clicked.connect(self.send_single_command)
        self.Power.clicked.connect(self.send_power_command)
        self.Stop.clicked.connect(self.send_stop_command)
        self.Reset.clicked.connect(self.reset)
        self.show()

    # ----------- BUTTON HANDLERS -----------
    def send_sweep_command(self):
        """Full sweep of frequencies from Pico"""
        self.plot_mode = "IV"
        self._voltage.clear()
        self._current.clear()
        self.plot_widget.reset_plot()
        if self._worker is None:
            self._worker = Connect("SWEEP\n")
        else:
            raise Exception("Worker already initialized...")
        self._worker.update_data.connect(self.update_plot)
        self._worker.finished_collecting.connect(self.on_data_finished)
        self._worker.start()

    def send_single_command(self):
        """Single I-V curve from Pico"""
        self.plot_mode = "IV"
        self._voltage.clear()
        self._current.clear()
        self.plot_widget.reset_plot()
        self._worker = Connect("SINGLE\n")
        self._worker.update_data.connect(self.update_plot)
        self._worker.finished_collecting.connect(self.on_data_finished)
        self._worker.start()

    def send_power_command(self):
        """Max power point tracker"""
        self.plot_mode = "MPPT"
        self._time.clear()
        self._power.clear()
        self.plot_widget.reset_plot()
        self._worker = Connect("MPPT\n")
        self._worker.update_data.connect(self.update_plot)
        self._worker.start()
    
    def send_stop_command(self):
        self.plot_mode = "MPPT"
        if self._worker is not None:
            print(f"Killing Worker... {self._worker}")
            self._worker.send_command("STOP\n")
            # self._worker.start()

    # ----------- DATA HANDLER -----------
    @pyqtSlot(str)
    def update_plot(self, data: str):
        print(data)
        parts = data.strip().split(",")
        # Frequency line: start of a new sweep
        if len(parts) == 2:
            try:
                _, freq_raw = parts
                freq = float(freq_raw)

                # Finalize previous frequency's data
                if self.previous_frequency is not None:
                    prev_data = self.freq_data[self.previous_frequency]
                    if 0.0 not in prev_data['voltage']:
                        isc = max(prev_data['current']) if prev_data['current'] else 0.0
                        prev_data['voltage'].append(0.0)
                        prev_data['current'].append(isc)
                        prev_data['voltage_corrected'].append(0.0)
                    self.parameters(np.array(prev_data['voltage']),
                                    np.array(prev_data['current']),
                                    freq=self.previous_frequency)
                    self.parameters(np.array(prev_data['voltage_corrected']),
                                    np.array(prev_data['current']),
                                    freq=self.previous_frequency)
                    
                self.current_frequency = freq
                self.previous_frequency = freq
                # Allocate data list and color
                if self.current_frequency not in self.freq_data:
                    self.freq_data[self.current_frequency] = {
                        'voltage': [],
                        'voltage_corrected': [],
                        'current': [],
                        'time': [],
                        'power': [],
                        'power_corrected': []
                    }
                print(f"Current Frequency: {self.current_frequency} Hz")
            except Exception as e:
                print(f"Invalid data format: {data} -> {e}")
            return
        
        elif self.plot_mode == "IV":
            if len(parts) == 7:
                time_raw, duty_raw, voltage_raw, voltage_corrected_raw, current_raw, power_raw, power_corrected_raw = parts
                time = float(time_raw)
                voltage = float(voltage_raw)
                voltage_corrected = float(voltage_corrected_raw)
                current = float(current_raw)
                power = float(power_raw)
                power_corrected = float(power_corrected_raw)

                if self.current_frequency not in self.freq_data:
                    self.freq_data[self.current_frequency] = {
                        'voltage': [],
                        'voltage_corrected': [],
                        'current': [],
                        'time': [],
                        'power': [],
                        'power_corrected': []
                    }

                # Avoid duplicates by point
                if self.current_frequency is not None:
                    v_list = self.freq_data[self.current_frequency]['voltage']
                    v_c_list = self.freq_data[self.current_frequency]['voltage_corrected']
                    i_list = self.freq_data[self.current_frequency]['current']
                    # Avoid duplicate points
                    if voltage_corrected in v_list or current in i_list:
                        return
                    v_list.append(voltage)
                    v_c_list.append(voltage_corrected)
                    i_list.append(current)
                    self.update_combined_plot()

        elif self.plot_mode == "MPPT":
            if len(parts) == 4:
                time_raw, duty_raw, power_raw, power_corrected_raw = parts
                time = float(time_raw)
                power = float(power_raw)
                power_corrected = float(power_corrected_raw)
                
                if self.current_frequency not in self.freq_data:
                    self.freq_data[self.current_frequency] = {
                        'voltage': [],
                        'voltage_corrected': [],
                        'current': [],
                        'time': [],
                        'power': [],
                        'power_corrected': []
                    }

                self.freq_data[self.current_frequency]['time'].append(time)
                self.freq_data[self.current_frequency]['power'].append(power)
                self.freq_data[self.current_frequency]['power_corrected'].append(power_corrected)
                self._time = self.freq_data[self.current_frequency]['time']
                self._power = self.freq_data[self.current_frequency]['power']
                self._power_corrected = self.freq_data[self.current_frequency]['power_corrected']
                self.update_combined_plot()
        else:
            print(f"Unexpected data format: {data}")
            return
        
    def update_combined_plot(self):
        self.plot_widget.axes.clear()
        colors = plt.get_cmap('tab10').colors

        for idx, (freq, data) in enumerate(self.freq_data.items()):
            color = colors[idx % len(colors)]
            label = f"{freq} Hz"

            if self.plot_mode == "IV":
                self.plot_widget.axes.set_ylabel("Current (A)")
                self.plot_widget.axes.set_xlabel("Voltage (V)")
                current = data.get('current', [])
                voltage = data.get('voltage', [])
                voltage_corrected = data.get('voltage_corrected', [])
                if voltage and current:
                    self.plot_widget.axes.plot(voltage, current, color=color, label=label)
                    self.plot_widget.axes.plot(voltage_corrected, current, color=color, linestyle='dashdot', label=f'corrected {label}')
                    self.plot_widget.axes.set_xlim(left=0)
                    self.plot_widget.axes.set_ylim(bottom=0)

            elif self.plot_mode == 'MPPT':
                self.plot_widget.axes.set_xlabel("Time (s)")
                self.plot_widget.axes.set_ylabel("Power (W)")
                time = data.get('time', [])
                power = data.get('power', [])
                power_corrected = data.get('power_corrected', [])
                if time and power:
                    self.plot_widget.axes.plot(time, power, color=color, label=label)
                    self.plot_widget.axes.plot(time, power_corrected, color=color, linestyle='dashdot', label=f'corrected {label}')
                    self.plot_widget.axes.set_xlim(left=0)
                    self.plot_widget.axes.set_ylim(bottom=0)

        self.plot_widget.axes.legend()
        self.plot_widget.draw()

    # ----------- RESET -----------
    def reset(self):
        self.plot_widget.reset_plot()
        self._voltage.clear()
        self._current.clear()
        self.results.clear()
        self.ResultsTable.clearContents()
        self.ResultsTable.setRowCount(0)
        self.plot_mode = "IV"

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
            v_c_arr = np.array(self.freq_data[self.current_frequency]['voltage_corrected'])
            i_arr = np.array(self.freq_data[self.current_frequency]['current'])
            # Add zero-voltage point if not already present
            if 0.0 not in self.freq_data[self.current_frequency]['voltage']:
                self.freq_data[self.current_frequency]['voltage'].append(0.0)
                self.freq_data[self.current_frequency]['current'].append(np.max(i_arr))
            if 0.0 not in self.freq_data[self.current_frequency]['voltage_corrected']:
                self.freq_data[self.current_frequency]['voltage_corrected'].append(0.0)

            self.update_combined_plot()
            self.parameters(v_arr, i_arr, freq=self.current_frequency)
            self.parameters(v_c_arr, i_arr, freq=self.current_frequency)
            # Save final data to CSV
            df = pd.DataFrame({
                'Current (A)': i_arr,
                'Voltage (V)': v_arr,
                'Voltage corrected (V)': v_c_arr
            })
            filename = f"sweep_{int(self.current_frequency)}Hz.csv"
            df.to_csv(filename, index=False)
            print(f"Saved sweep to {filename}")

# ----------- MAIN LAUNCHER -----------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
