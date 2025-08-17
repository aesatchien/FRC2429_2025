from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QBrush, QPen, QColor
from ntcore import NetworkTableInstance
from pathlib import Path
from datetime import datetime
import os
import csv
from wpimath.filter import Debouncer
import math
# from datetime import datetime
class Ui(QtWidgets.QMainWindow):
    # set the root dir for the project, knowing we're one deep
    root_dir = Path('.').absolute()  # set this to be in the root, not a child, so no .parent. May need to change this.
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    # -------------------  INIT  --------------------------
    def __init__(self):
        super(Ui, self).__init__()
        # trick to inherit all the UI elements from the design file  - DO NOT CODE THE LAYOUT!
        uic.loadUi('dashboard.ui', self)  # if this isn't in the directory, you got no program

        # set up network tables - TODO need to really see which of these is necessary
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"] #  "roboRIO-2429-FRC.local"]  # need to add the USB one here
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0  # manually do a round-robin later
        # self.ntinst.setServer("127.0.0.1",0)
        #self.ntinst.setServer(servers=self.servers)  # does not seem to work in round-robin in 2023 code
        self.ntinst.setServerTeam(2429)
        self.connected = self.ntinst.isConnected()

        self.table = self.ntinst.getTable("datatable")
        self.control_mode_publisher = self.table.getStringTopic("control_mode").publish()
        self.control_mode_subscriber = self.table.getStringTopic("control_mode").subscribe('onboard')
        self.translation_limit_publisher = self.table.getDoubleTopic("thrust_limit").publish()
        self.twist_limit_publisher = self.table.getDoubleTopic("twist_limit").publish()

        self.initialize_widgets()


        self.counter = 1

        self.refresh_time = 21  # milliseconds before refreshing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)

        self.show()

    def initialize_widgets(self):
        self.qbutton_swap_sim.clicked.connect(self.increment_server)
        self.qbutton_control_mode_switch.clicked.connect(self.switch_control_modes)
        self.qslider_translation.setMinimum(1)
        self.qslider_translation.setMaximum(10)
        self.qslider_twist.setMinimum(1)
        self.qslider_twist.setMaximum(10)
        self.qslider_translation.valueChanged.connect(lambda: self.translation_limit_publisher.set(self.qslider_translation.value()/10))
        self.qslider_twist.valueChanged.connect(lambda: self.twist_limit_publisher.set(self.qslider_twist.value()/10))

    def switch_control_modes(self):
        if self.control_mode_subscriber.get() == 'remote':
            print('setting to onboard')
            self.control_mode_publisher.set('onboard')
        else:
            print('setting to remote')
            self.control_mode_publisher.set('remote')

    def update_widgets(self):
        self.counter += 1

        if self.ntinst.isConnected():
            self.qlabel_nt_connected.setStyleSheet('color: rgb(50, 255, 50)')
            self.qlabel_nt_connected.setText(f'nt connected to {self.servers[self.server_index]}')
        else:
            # todo: fix.  This is broken in the main dashboard and mine- it never goes back to being disconnected.
            self.qlabel_nt_connected.setStyleSheet('color: red')
            self.qlabel_nt_connected.setText('nt disconnected')

        if self.control_mode_subscriber.get() == 'onboard':
            self.qbutton_control_mode_switch.setText(f'switch to remote control')
        else:
            self.qbutton_control_mode_switch.setText('switch to onboard control')

        self.qlabel_twist_percent.setText(f'{self.qslider_twist.value() * 10:.0f}%')
        self.qlabel_translation_percent.setText(f'{self.qslider_translation.value() * 10:.0f}%')

    def increment_server(self):  # changes to next server in server list - TODO - figure our how to make this immediate
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])

        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')
        # self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {self.servers[self.server_index]} ... wait 5s')

if __name__ == "__main__":
    import sys

    # attempt to set high dpi scaling if it is detected - possible fix for high-def laptop and destop displays
    # if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    #     QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    # compensate for dpi scaling way up above with the setAttribute calls - don't really need this now
    screen = app.screens()[0]
    dpi_logical = int(screen.logicalDotsPerInchX())
    if dpi_logical > 96:  # 150% scaling on AVIT North, vs 96 for unscaled
        print(f"We're on a scaled screen: logical dpi is {dpi_logical}")
    else:
        print(f"We're not on a scaled screen: logical dpi is {dpi_logical}")

    ui = Ui()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Still has garbage collection issues. Closing.')
