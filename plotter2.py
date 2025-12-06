import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QPushButton, QSpacerItem, QSizePolicy
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QTimer
import subprocess
import signal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
import numpy as np
import time
import os

### WiFi LISTENER
import subprocess, re


### Pluto SDR
import adi

class FullScreenApp(QMainWindow):
    def check_pluto_connected(self):
        # ping Pluto once, discard output
        response = os.system("ping -c 1 -W 1 192.168.2.1 > /dev/null 2>&1")
        return response == 0
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pokritost signala - Heatmap Plotter")
        self.setGeometry(0, 0, 1024, 600)
        self.setStyleSheet("background-color: #ffffff")

        # Main central widget and vertical layout
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        self.main_layout.addSpacing(15)

        # Header label
        self.header_label = QLabel("Božo - Heatmap Plotter")
        self.header_label.setAlignment(Qt.AlignCenter)
        font = QFont("Univerza Sans", 20)
        self.header_label.setFont(font)
        self.main_layout.addWidget(self.header_label)
        self.main_layout.addSpacing(25)

        # Horizontal layout for left and right panels
        self.horizontal_layout = QHBoxLayout()
        self.main_layout.addLayout(self.horizontal_layout)

        # ---------------- LEFT PANEL ----------------
        self.left_widget = QWidget()
        self.left_widget.setFixedWidth(400)
        self.left_layout = QVBoxLayout(self.left_widget)
        self.left_layout.setContentsMargins(50, 0, 0, 0)

        # Left panel labels
        self.greeting_label = QLabel("Konfiguracija Pluto SDR")
        self.greeting_label.setAlignment(Qt.AlignLeft)        # Align text to the left
        #self.greeting_label.setContentsMargins(50, 0, 0, 0)  # Add 15 pixels margin from the left
        font = QFont("Univerza Sans", 16)
        self.greeting_label.setFont(font)
        self.left_layout.addWidget(self.greeting_label)

        self.pluto_on = QLabel("")
        self.pluto_on.setAlignment(Qt.AlignLeft)        # Align text to the left
        #self.greeting_label.setContentsMargins(50, 0, 0, 0)  # Add 15 pixels margin from the left
        font = QFont("Univerza Sans", 10)
        self.pluto_on.setFont(font)
        self.left_layout.addWidget(self.pluto_on)

        # Frequency input
        self.freq_label = QLabel("Frekvenca [Hz]")
        self.freq_label.setAlignment(Qt.AlignLeft)
        #self.freq_label.setContentsMargins(50, 0, 0, 10)
        self.left_layout.addSpacing(10)
        font = QFont("Univerza Sans", 10)
        self.freq_label.setFont(font)
        self.left_layout.addWidget(self.freq_label)

        self.freq_input = QLineEdit()
        self.freq_input.setAlignment(Qt.AlignLeft)
        #self.freq_input.setContentsMargins(50, 0, 0, 15)
        input_font = QFont("Univerza Sans", 10)  # 16 pt font, adjust as needed
        self.freq_input.setFont(input_font)
        self.freq_input.setText("864e6")  # default value
        self.freq_input.setFixedWidth(250)
        self.left_layout.addWidget(self.freq_input)

        # Bandwidth input
        self.band_label = QLabel("Bandwidth [Hz]")
        self.band_label.setAlignment(Qt.AlignLeft)
        #self.band_label.setContentsMargins(50, 0, 0, 0)
        self.left_layout.addSpacing(5)
        self.band_label.setFont(font)
        self.left_layout.addWidget(self.band_label)

        self.band_input = QLineEdit()
        self.band_input.setAlignment(Qt.AlignLeft)
        #self.band_input.setContentsMargins(50, 0, 0, 15)
        self.band_input.setFont(input_font)
        self.band_input.setText("2.4e6")  # default value
        self.band_input.setFixedWidth(250)
        self.band_input.setAlignment(Qt.AlignLeft)
        self.left_layout.addWidget(self.band_input)

        # Sample input
        self.samp_label = QLabel("Sample rate [Hz]")
        self.samp_label.setAlignment(Qt.AlignLeft)
        self.left_layout.addSpacing(5)
        #self.samp_label.setContentsMargins(50, 0, 0, 0)
        self.samp_label.setFont(font)
        self.left_layout.addWidget(self.samp_label)

        self.samp_input = QLineEdit()
        self.samp_input.setAlignment(Qt.AlignLeft)
        #self.samp_input.setContentsMargins(50, 0, 0, 15)
        self.samp_input.setFont(input_font)
        self.samp_input.setText("2.4e6")  # default value
        self.samp_input.setFixedWidth(250)
        self.samp_input.setAlignment(Qt.AlignLeft)
        self.left_layout.addWidget(self.samp_input)

        #------------------CHECK PLUTO CONNECTION-------------------
        if not self.check_pluto_connected():
        # Disable inputs if Pluto not connected
            self.freq_input.setDisabled(True)
            self.band_input.setDisabled(True)
            self.samp_input.setDisabled(True)
            self.pluto_on.setText("Pluto ni priključen (Wi-Fi meritev)")
            self.not_connected = True
        else:
            self.pluto_on.setText("Pluto je priključen")
            self.not_connected = False

        # ---------------- START MEASUREMENT BUTTON ----------------
        self.start_button = QPushButton("Začni meritve")
        button_font = QFont("Univerza Sans", 10)
        self.start_button.setFont(button_font)
        self.left_layout.addSpacing(10)
        self.start_button.setFixedSize(250, 50)
        self.start_button.setStyleSheet("background-color: none")
        self.start_button.clicked.connect(self.start_measurement)
        self.left_layout.addWidget(self.start_button)

        self.stop_button = QPushButton()
        self.stop_requested = False
        self.left_layout.addSpacing(15)
        icon = QIcon("./Stop_sign.png")  # Replace "example.png" with the path to your image
        icon_actual = icon.actualSize(QSize(75, 75))  # Set the size of the icon
        self.stop_button.setFixedWidth(250)
        self.stop_button.setStyleSheet("border: none;")
        self.stop_button.setIcon(icon)
        self.stop_button.setIconSize(icon_actual)  # Set icon size to specified size
        self.stop_button.clicked.connect(self.stop_clicked)  # Connect button click signal to image_clicked function
        self.left_layout.addWidget(self.stop_button)

        self.close_button = QPushButton("Zapri")
        button_font = QFont("Univerza Sans", 10)
        self.close_button.setFont(button_font)
        self.left_layout.addSpacing(15)
        self.close_button.setFixedSize(250, 50)
        self.close_button.setStyleSheet("background-color: none")
        self.close_button.clicked.connect(self.close_program)
        self.left_layout.addWidget(self.close_button)

        # Spacer to push content to the top
        self.left_layout.addStretch()

        # Add left panel to horizontal layout
        self.horizontal_layout.addWidget(self.left_widget)

        # ---------------- RIGHT PANEL (placeholder) ----------------
        self.right_widget = QWidget()
        self.right_layout = QVBoxLayout(self.right_widget)
        self.right_layout.setSpacing(5)                # spacing between widgets
        self.right_layout.setContentsMargins(0, 0, 0, 0)


        # You can add images, buttons, or other widgets here
        self.right_placeholder = QLabel("Heat map")
        self.right_placeholder.setAlignment(Qt.AlignCenter)
        self.right_layout.setAlignment(Qt.AlignTop)
        font = QFont("Univerza Sans", 16)
        self.right_placeholder.setFont(font)
        self.right_layout.addWidget(self.right_placeholder)
        #self.right_layout.addSpacing(0)  # reduce space
        #-----------------PLOT------------------------------------
        self.fig = Figure(figsize=(5, 5))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.cbar = None
        self.right_layout.addWidget(self.canvas)
        self.horizontal_layout.addWidget(self.right_widget)

    #---------------- STOP MEASUREMENT ----------------
    def stop_clicked(self):
        print("Stopping measurement...")
        self.start_button.setStyleSheet("background-color: none")
        self.stop_requested = True

    #---------------- CLOSE PROGRAM ----------------
    def close_program(self):
        print("Closing program...")
        self.close()

    #---------------- START MEASUREMENT ----------------
    def start_measurement(self):
        if  self.not_connected:
            self.start_button.setStyleSheet("background-color: green")
            print("Wi-fi meritev")
        else: 
            self.start_button.setStyleSheet("background-color: green")
            print("Starting measurement...")
            rx_buffer_size = 1024
            # Read parameters from GUI
            freq = float(self.freq_input.text())
            bw   = float(self.band_input.text())
            samp = float(self.samp_input.text())
            self.x = 0.0
            self.y = 0.0
            self.amps = 0.0
            print("Initializing Pluto SDR...")
            try:
                self.sdr = adi.Pluto("ip:192.168.2.1")
                self.sdr.rx_lo = int(freq)    # Frequency in Hz
                self.sdr.sample_rate = int(samp)
                self.sdr.rx_rf_bandwidth = int(bw)
                self.sdr.gain_control_mode_chan0 = "manual"
                self.sdr.rx_hardwaregain_chan0 = 45
                self.sdr.rx_buffer_size = rx_buffer_size
                print("Pluto SDR initialized.")
            except Exception as e:
                print("ERROR initializing Pluto:", e)
                return
            
            ##-----ROS initialize-----
            rclpy.init()
            self.ros_node = rclpy.create_node("qt_heatmap_node")
            self.subscription = self.ros_node.create_subscription(
                TFMessage,
                '/locobot/mobile_base/tf',
                self.pose_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.timestamp = time.time()
            self.ros_timer.start(20)  # start live updates

    #---------------- UPDATE LIVE PLOT ----------------
    def update_live_plot(self):
        if self.not_connected == False and not self.check_pluto_connected(): 
            print("Pluto disconnected during measurement. Stopping and closing.")
            self.timer.stop()
            self.close()
            return
        # --- Receive samples ---
        samples = self.sdr.rx()
        # --- Split I/Q ---
        I = np.real(samples)/2048
        Q = np.imag(samples)/2048
        P = np.mean((I**2 + Q**2))
        P_dB = int(10 * np.log10(P))
        self.amps = P_dB
        print("Power dB: ", P_dB)
        if self.cbar is not None:
            self.cbar.remove()
        self.ax.clear()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        scatter = self.ax.scatter(np.array([self.x]), np.array([self.y]), c=np.array([self.amps]), cmap='plasma', vmin=-100, vmax=10, marker='o')
        self.cbar = self.fig.colorbar(scatter, ax=self.ax)
        self.cbar.set_label("Power [dB]")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.canvas.draw()

    def ros_spin(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    def pose_callback(self, msg):
        if self.stop_requested:
            self.ros_node.destroy_node()
            rclpy.shutdown()
            self.ros_timer.stop()
            return

        # Extract the first transform from the TFMessage
        if len(msg.transforms) == 0:
            return  # No transforms available

        self.x = msg.transforms[0].transform.translation.x
        self.y = msg.transforms[0].transform.translation.y

        if self.timestamp + 0.1 < time.time():  # Update every 0.5 seconds
            self.timestamp = time.time()
            self.update_live_plot()
            # Get signal amplitude
    
def main(args=None):
    
    # rclpy.init()
    # heatmap_plotter = HeatmapPlotter()
    # rclpy.spin(heatmap_plotter)
    
    # heatmap_plotter.destroy_node()
    # rclpy.shutdown()
    app = QApplication(sys.argv)
    window = FullScreenApp()
    window.showFullScreen()  # Show the window in fullscreen mode
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
