# -*- coding: utf-8 -*-
"""
Created on Thu May  8 09:32:50 2025

@author: ruiui
"""

import serial
import pandas as pd
import logging
import time
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLayout, QVBoxLayout, QWidget
from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

# --- CONFIGURATION ---
FILE_actual = "4V_Light.txt"
FILENAME = 'iv_data_4V_RCC_solder.csv'
# FILENAME = 'iv_data_4V_5_40k.csv'
SVG = '4V_graph_solder.svg'
# FILENAMEA = "2V_Light.txt"
# FILENAME = 'iv_data_2V_RC.csv'
# FILENAME = 'iv_data_2V_RCC.csv'
# FILENAME = 'iv_data_4V_5_40k.csv'
# SVG = '2V_graph.svg'
# FILENAMEA = "1V_Light.txt"
# FILENAME = 'iv_data_1V_1_RC.csv'
# FILENAME = 'iv_data_1V_1_RCC.csv'
# FILENAME = 'iv_data_1V_1_60k.csv'
# FILENAME = 'iv_data_1V_1_80k.csv'
# SVG = '1V_graph.svg'
# FILENAMEA = "05V_Light.txt"
# FILENAME = 'iv_data_05V_RC.csv'
# FILENAME = 'iv_data_05V_RCC.csv'
# FILENAME = 'iv_data_05V_60k.csv'
# SVG = '05V_graph.svg'

# --- LOAD ACTUAL ---
# Skip the first 6 rows of metadata
df_actual = pd.read_csv(FILE_actual, skiprows=9, delim_whitespace=True)

df_actual = df_actual.dropna(subset=['Vmeas', 'Imeas'])
df_actual['Vmeas'] = pd.to_numeric(df_actual['Vmeas'], errors='coerce')
df_actual['Imeas'] = pd.to_numeric(df_actual['Imeas'], errors='coerce')

# Drop any remaining NaNs
df_actual = df_actual.dropna()

voltage_actual = df_actual['Vmeas']
current_actual = df_actual['Imeas']

# --- SERIAL CONNECTION ---
PORT = 'COM11'
BAUD = 115200
results = []

def is_data_line(line):
    parts = line.strip().split(',')
    if len(parts) != 6:
        return False
    try:
        [float(p) for p in parts]
        return True
    except ValueError:
        return False
    
class FigWidget(QWidget):
    """Class to create a new window with Plots"""
    def __init__(self, title: str):
        logging.info("FigWidget window added to VBOXLayout")
        super().__init__()
        self.setWindowTitle(title)
        self.layout: QLayout = QVBoxLayout()
        self.setLayout(self.layout)

class PltFigure(FigureCanvasQTAgg):
    """
    Class to draw canvas for a particular figure
    Args:
        parent: parent widget where the figure will be drawn
        xlabel, ylabel: Labels for x and yaxis
        width, height, dpi: Image size/resolution
        interactive: wheter to add interactive vertical guideline
    Properties:
        figBuffer: Holder for the figure buffer (for interactive plots)
    Methods:
        draw_axes: Redraw axes elements (given a xlabel and ylabel)
        reinit: Clean an reinitialize figure elements
        reset_figBuffer: Update the figure buffer (for when new lines are added)
        fast_draw: Fast draw line elements onto the plot
        bufferRedraw: wrapper around draw to also save the figure to the buffer
        deleteLinesGID: delete lines from the plot from their gid
    """

    def __init__(
        self, parent, xlabel, ylabel, width=6, height=5, dpi=100, interactive=True):
        """Initialize all the figure main elements"""
        logging.info("Initialize Figure Canvas")
        self.xlabel = xlabel
        self.ylabel = ylabel
        self._vline = None
        self._vlineLock = True
        self._fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self._fig.add_subplot(111)
        self.draw_axes()
        super(PltFigure, self).__init__(self._fig)
        parent.addWidget(self)
        # Variable to store the figure buffer for fast redraw
        self._fig_buffer = self.copy_from_bbox(self._fig.bbox)

    """ Initialization functions """

    def draw_axes(self, xlabel=None, ylabel=None):
        """Update x and ylabels for the plot"""
        logging.debug("Draw/Labelling Axis")
        if xlabel:
            self.xlabel: str = xlabel
        if ylabel:
            self.ylabel: str = ylabel
        # self.axes.yaxis.grid(True, linestyle="--")
        # self.axes.xaxis.grid(True, linestyle="--")
        self.axes.grid(False)
        self.axes.set_ylabel(self.ylabel)
        self.axes.set_xlabel(self.xlabel)

    def reinit(self):
        """Clean and then reinit the figure elements"""
        logging.debug("Reinitialize the plot")
        self.axes.clear()
        self.draw_axes()
        self.bufferRedraw()

    """ Figure Properties """

    @property
    def figBuffer(self):
        return self._fig_buffer

    @figBuffer.setter
    def figBuffer(self, value):
        self._fig_buffer = value

    """ Figure Buffer properties """

    def reset_figBuffer(self):
        """
        Overwrite the figBuffer variable to accommodate new plot changes
        Also considers and ignores the vertical guideline
        """
        # Check for the vertical guideline to avoid storing it in the buffer
        if self._vline is not None:
            xdata = self._vline.get_xdata()[0]
            logging.debug(f"Vline: {xdata}")
            self.deleteLinesGID(["vline"])
            self._vline = self.axes.axvline(
                xdata,
                linestyle="--",
                color="k",
                picker=True,
                gid="vline",
                pickradius=15,
            )
            self._fig_buffer = self.copy_from_bbox(self._fig.bbox)
            self.fast_draw([self._vline])
        else:
            self._fig_buffer = self.copy_from_bbox(self._fig.bbox)

    def fast_draw(self, lines):
        """Fast draw line elements onto the canvas"""
        self._fig.canvas.restore_region(self._fig_buffer)
        for line in lines:
            self.axes.draw_artist(line)
        self._fig.canvas.blit(self._fig.bbox)
        self._fig.canvas.flush_events()

    def bufferRedraw(self):
        """Wrapper around the draw function to also run reset_figBuffer"""
        logging.debug("Rebuilding buffer cache")
        self.draw()
        self.reset_figBuffer()

    """ Interaction functions """

    def deleteLinesGID(self, lines):
        """Delete lines based on gid"""
        for line in self.axes.get_lines():
            if line.get_gid() in lines:
                logging.debug(f"Removing line: {line.get_gid()}")
                line.remove()
        self.draw()

class Connect(QThread):
    update_data = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        
    def run(self):
        print("Starting worker thread...")
        with serial.Serial('COM11', 115200, timeout=1) as conn:
            conn.write(b"START\n")
            while True:
                # Wait for the measurement results from the pico
                if conn.in_waiting > 0:
                    line = conn.readline().decode(errors="ignore").strip()
                    # print(line)
                    # When the pico finishes it will send END for the script to finish
                    if line == "END":
                        break
                    self.update_data.emit(line)
        time.sleep(0.5)
        print("Finished Tread")

class GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # Create Layout and add and configure graph
        self.layout: QLayout = QVBoxLayout()
        self.figure = PltFigure(self.layout, "Voltage", "Current")
        self.setCentralWidget(self.figure)
        self.axes = self.figure.axes
        self.figure.draw_axes("Voltage", "Current")
        self.plot_data = self.axes.plot([0], [0])
        self.plot_data[0].set_ydata(current_actual)
        self.plot_data[0].set_xdata(voltage_actual)
        self.axes.set_xlim(0, 4)
        self.axes.set_ylim(0, 0.2)
        self.figure.bufferRedraw()
        self._voltage = []
        self._current = []
        self.show()
        # Create worker thread
        self.start_worker()       
        
    def start_worker(self):
        self.worker = Connect()
        self.worker.update_data.connect(self.update_plot)
        self.worker.start()
    
    @pyqtSlot(str)
    def update_plot(self, data: str):
        print(data)
        _, _, _, voltage, current, _ = tuple(data.split(","))
        self._voltage.append(float(voltage))
        self._current.append(float(current))
        self.plot_data[0].set_ydata(self._current)
        self.plot_data[0].set_xdata(self._voltage)
        self.figure.fast_draw(self.plot_data)
        
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    interface = GUI()
    sys.exit(app.exec_())
    




# while True:
#     #Fazer os gráficos
#     # Load the CSV file
#     df_raw = pd.read_csv(FILENAME)
#     df_raw['Frequency (Hz)'] = pd.to_numeric(df_raw['Frequency (Hz)'], errors='coerce')
#     frequencies = df_raw['Frequency (Hz)'].unique()
#     plt.figure(figsize=(10, 6))
#     for freq in frequencies:
#         df_freq = df_raw[df_raw['Frequency (Hz)'] == freq]
#         directions = df_freq['Direction'].unique()
#         for direction in directions:
#             subset = df_freq[df_freq['Direction'] == direction]
#             # Drop duplicates
#             subset = subset.drop_duplicates(subset=['Voltage (V)'])
#             subset = subset.drop_duplicates(subset=['Current (A)'])

#             if subset.empty or len(subset) < 10:
#                 continue
#             # --- DATA TREATMENT ---
#             idx_max = subset['Current (A)'].idxmax()
#             max_point = subset.loc[idx_max]
            
#             new_row = max_point.copy()
#             new_row['Voltage (V)'] = 0
#             new_row['Current (A)'] *= 1
#             new_row['Power (W)'] = 0
#             new_row['Duty (%)'] = df_raw['Duty (%)'].max() + 0.1

#             subset = pd.concat([subset, new_row.to_frame().T], ignore_index=True)
#             subset = subset.sort_values('Voltage (V)')
            
#             voltage = subset['Voltage (V)'].values
#             current = subset['Current (A)'].values * 1000
#             # --- PLOT ---
#             plt.plot(voltage,current, label='raw')
#             plt.plot(voltage_actual, current_actual, label='actual')

#             plt.xlabel('Voltage (V)', fontsize=15)
#             plt.ylabel('Current (mA)', fontsize=15)
#             plt.title('I-V Curves', fontsize=18)
#             plt.xticks(fontsize=12)
#             plt.yticks(fontsize=12)
#             plt.legend(fontsize=13)
#             plt.grid(False)
#     # Save as SVG
#     plt.savefig(SVG, format='svg')
#     plt.show()



# with serial.Serial(PORT, BAUD, timeout=1) as ser, open(FILENAME, 'w') as f:
#     print(f"Connected to {PORT}. Logging to {FILENAME}...")

#     # Initialize 'recording' variable
#     recording = False

#     f.write("Frequency (Hz),Direction,Duty (%),Voltage (V),Current (A),Power (W)\n")

#     try:
#         while True:
#             line = ser.readline().decode(errors='ignore').strip()

#             # Check for the BEGIN marker from the Pico
#             if line == "BEGIN":
#                 recording = True
#                 print("Started logging data...")
#                 continue

#             # Start recording only after receiving BEGIN
#             if recording and is_data_line(line):
#                 print(line)
#                 f.write(line + '\n')

#             if line == "Done!":
#                 # Flush write buffer to ensure all data is on disk
#                 f.flush()
#                 os.fsync(f.fileno())

#                 # Load the CSV file
#                 df_raw = pd.read_csv(FILENAME)

#                 df_raw['Frequency (Hz)'] = pd.to_numeric(df_raw['Frequency (Hz)'], errors='coerce')

#                 frequencies = df_raw['Frequency (Hz)'].unique()

#                 plt.figure(figsize=(10, 6))

#                 for freq in frequencies:
#                     df_freq = df_raw[df_raw['Frequency (Hz)'] == freq]
#                     directions = df_freq['Direction'].unique()
                    
#                     for direction in directions:
#                         subset = df_freq[df_freq['Direction'] == direction]

#                         # Drop duplicates
#                         subset = subset.drop_duplicates(subset=['Voltage (V)'])
#                         subset = subset.drop_duplicates(subset=['Current (A)'])

#                         if subset.empty or len(subset) < 10:
#                             continue

#                         # --- DATA TREATMENT ---
#                         idx_max = subset['Current (A)'].idxmax()
#                         max_point = subset.loc[idx_max]
                        
#                         new_row = max_point.copy()
#                         new_row['Voltage (V)'] = 0
#                         new_row['Current (A)'] *= 1
#                         new_row['Power (W)'] = 0
#                         new_row['Duty (%)'] = df_raw['Duty (%)'].max() + 0.1

#                         subset = pd.concat([subset, new_row.to_frame().T], ignore_index=True)
#                         subset = subset.sort_values('Voltage (V)')
                        
#                         voltage = subset['Voltage (V)'].values
#                         current = subset['Current (A)'].values * 1000
                        
#                         # --- PARAMETER EXTRACTION ---
#                         # Voc
#                         voc_idx_closest_zero = np.argmin(np.abs(current))
#                         voc = voltage[voc_idx_closest_zero]
#                         # Isc
#                         isc_idx_closest_zero = np.argmin(np.abs(voltage))
#                         isc = current[isc_idx_closest_zero]
#                         # MPP
#                         power = voltage * current
#                         pmax_idx = np.argmax(power)
#                         vmp = voltage[pmax_idx]
#                         imp = current[pmax_idx]
#                         pmax = power[pmax_idx]
                        
#                         # MPP
#                         current_actual = df_actual['Imeas']*1000
#                         power_actual = voltage_actual * current_actual
#                         pmax_idx_actual = np.argmax(power_actual)
#                         vmp_actual = voltage_actual[pmax_idx_actual]
#                         imp_actual = current_actual[pmax_idx_actual]
#                         pmax_actual = power_actual[pmax_idx_actual]

#                         mpp_eff = (1-(pmax_actual-pmax)/pmax_actual)*100
                        
#                         # Rs
#                         if len(voltage) >= 5:
#                             rs_fit = np.polyfit(current[-10:], voltage[-10:], 1)
#                             rs = -rs_fit[0]
#                         else:
#                             rs = np.nan
                        
#                         # Rsh
#                         if len(voltage) >= 5:
#                             rsh_fit = np.polyfit(current[:10], voltage[:10], 1)
#                             rsh = -rsh_fit[0]
#                         else:
#                             rsh = np.nan
                        
#                         # Save results
#                         results.append({
#                             'Frequency (Hz)': freq,
#                             'Direction': int(direction),
#                             'Voc (V)': round(voc, 3),
#                             'Isc (A)': round(isc, 3),
#                             'Vmp (V)': round(vmp, 3),
#                             'Imp (A)': round(imp, 3),
#                             'Pmax (W)': round(pmax, 3),
#                             'Rs (Ω)': round(rs, 2),
#                             'Rsh (Ω)': round(rsh, 2)
#                         })
                        
#                         # --- PLOT ---
#                         plt.plot(voltage,current, label='raw')
#                         plt.plot(voltage_actual, current_actual, label='actual')

#                 plt.xlabel('Voltage (V)', fontsize=15)
#                 plt.ylabel('Current (mA)', fontsize=15)
#                 plt.title('I-V Curves', fontsize=18)
#                 plt.xticks(fontsize=12)
#                 plt.yticks(fontsize=12)
#                 plt.ylim(bottom=0, top=isc*1.1)
#                 plt.xlim(left=0, right=voc*1.1)
#                 plt.legend(fontsize=13)
#                 plt.grid(False)

#                 # Create table data
#                 col_labels = ["","Imp (mA)", "Vmp (V)", "Pmax (mW)", "MPP efficiency (%)"]
#                 table_data = [
#                     ["Actual",f"{imp_actual:.2f}", f"{vmp_actual:.3f}", f"{pmax_actual:.2f}", f"{mpp_eff:.2f}"],
#                     ["Raw",f"{imp:.2f}", f"{vmp:.3f}", f"{pmax:.2f}", ""]
#                 ]

#                 # Create the table
#                 table = plt.table(cellText=table_data,
#                                   colLabels=col_labels,
#                                   cellLoc='center',
#                                   colLoc='center',
#                                   rowLoc='center',
#                                   loc='lower left',
#                                   bbox = [0.02, 0.045, 0.65, 0.35])

#                 col_widths = [0.1, 0.125, 0.125, 0.125, 0.2]

#                 # Loop over columns and cells to set width
#                 for col_idx, width in enumerate(col_widths):
#                     for row_idx in range(len(table_data)+1):
#                         cell = table[row_idx, col_idx]
#                         cell.set_width(width)

#                 table.auto_set_font_size(False)
#                 table.set_fontsize(12)

#                 plt.tight_layout()

#                 # Save as SVG
#                 plt.savefig(SVG, format='svg')

#                 plt.show()

#                 # --- TABLE ---
#                 results_df = pd.DataFrame(results)

#                 # Print table using tabulate
#                 print(tabulate(results_df, headers='keys', tablefmt='pretty'))

#     except KeyboardInterrupt:
#         print("\nLogging stopped.")
