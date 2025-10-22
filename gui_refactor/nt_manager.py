
import time
from datetime import datetime
from ntcore import NetworkTableInstance

class NTManager:
    def __init__(self, ui):
        self.ui = ui
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"]
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0
        self.ntinst.setServerTeam(2429)

    def isConnected(self):
        return self.ntinst.isConnected()

    def getEntry(self, key):
        return self.ntinst.getEntry(key)

    def getEntries(self, prefix, types):
        return self.ntinst.getEntries(prefix, types)

    def flush(self):
        self.ntinst.flush()

    def reconnect(self):
        sleep_time = 0.1
        self.ntinst.stopClient()
        time.sleep(sleep_time)
        self.ntinst.disconnect()
        time.sleep(sleep_time)
        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.ntinst.setServerTeam(2429)
        time.sleep(sleep_time)
        self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Reconnected to NetworkTables.')

    def increment_server(self):
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        next_server = self.servers[self.server_index]
        self.ntinst.setServer(server_name=next_server)
        print(f'Changed server from {current_server} to {next_server}')
        self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {next_server} ... wait 5s')

    def report_nt_status(self):
        if self.isConnected():
            try:
                connection = self.ntinst.getConnections()[0]
                id, ip = connection.remote_id, connection.remote_ip
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: id={id}, ip={ip}')
            except IndexError:
                 self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: Connected, but no connection info.')
        else:
            self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: Disconnected')
