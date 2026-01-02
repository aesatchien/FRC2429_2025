
import time
import urllib.request
from datetime import datetime
import cv2
from PyQt6.QtCore import QObject, pyqtSignal, QThread, Qt
from PyQt6.QtGui import QImage, QPixmap
from config import DEFAULT_CAMERA

class CameraWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, qtgui):
        super().__init__()
        self.qtgui = qtgui
        self.running = False
        self.frames = 0
        self.previous_read_time = 0
        self.max_fps = 20
        self.error_count = 0
        self.cv_backend = cv2.CAP_FFMPEG

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        old_url = ''
        cap = None
        try:
            while self.running:
                if self.qtgui.qradiobutton_autoswitch.isChecked():  # auto determine which camera to show
                    # you can change the logic here - e.g. when elevator is low or shooter is on, choose another camera
                    # shooter_on = self.qtgui.widget_dict['qlabel_shooter_indicator']['entry'].getBoolean(False)
                    # elevator_low = self.qtgui.widget_dict['qlcd_elevator_height']['entry'].getDouble(100) < 100
                    
                    if DEFAULT_CAMERA in self.qtgui.camera_dict:
                        url = self.qtgui.camera_dict[DEFAULT_CAMERA]['URL']
                    else:
                        time.sleep(0.1)
                        continue
                else:
                    current_cam = self.qtgui.qcombobox_cameras.currentText()
                    if current_cam in self.qtgui.camera_dict:
                        url = self.qtgui.camera_dict[current_cam]['URL']
                    else:
                        time.sleep(0.1)
                        continue

                if url != old_url:
                    if cap is not None:
                        cap.release()

                    retries = 0
                    max_retries = 10
                    while retries < max_retries:
                        cap = cv2.VideoCapture(url, self.cv_backend)
                        if cap.isOpened():
                            print(f"Successfully opened stream: {url}")
                            break
                        else:
                            retries += 1
                            print(f"Cannot open stream: {url}. Retrying ({retries}/{max_retries})...")
                            time.sleep(0.5)
                    
                    if not cap or not cap.isOpened():
                        print(f"Failed to open stream after {max_retries} retries. Worker thread exiting.")
                        return # Exit the thread

                    old_url = url

                now = time.time()
                if now - self.previous_read_time > 1.0 / self.max_fps:
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            self.frames += 1
                            pixmap = self.qtgui.convert_cv_qt(frame, self.qtgui.qlabel_camera_view)
                            self.qtgui.qlabel_camera_view.setPixmap(pixmap)
                            self.previous_read_time = now
                        else:
                            print("[ERROR] OpenCV stopped receiving frames. Retrying...")
                            cap.release()
                            cap = cv2.VideoCapture(url, self.cv_backend) # Attempt to re-open
                    else:
                        time.sleep(0.5) # Wait if cap is not ready
                else:
                    if cap.isOpened():
                        cap.grab() # Flush buffer

        except Exception as e:
            print(f'{datetime.today().strftime("%H%M%S")} Camera worker crashed: {e}')
        finally:
            if cap is not None:
                cap.release()
            self.finished.emit()

class CameraManager:
    def __init__(self, ui):
        self.ui = ui
        self.thread = None
        self.worker = None

    def check_url(self, url):
        try:
            code = urllib.request.urlopen(url, timeout=0.2).getcode()
            return code == 200
        except Exception as e:
            print(f'Failure: attempted to check {url} with exception {e}')
        return False

    def toggle_camera_thread(self):
        live_cams = [self.check_url(d['URL']) for d in self.ui.camera_dict.values() if 'URL' in d]

        if any(live_cams):
            if self.thread is None:
                self.thread = QThread()
                self.worker = CameraWorker(qtgui=self.ui)
                self.worker.moveToThread(self.thread)
                self.thread.started.connect(self.worker.run)
                self.worker.finished.connect(self.thread.quit)
                self.thread.start()
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Starting camera thread')
            elif not self.thread.isRunning():
                # If the thread has finished, we may need to create a new worker
                self.worker = CameraWorker(qtgui=self.ui)
                self.worker.moveToThread(self.thread)
                self.thread.started.connect(self.worker.run)
                self.worker.finished.connect(self.thread.quit)
                self.thread.start()
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Restarting camera thread')
            else:
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Camera thread already running.')
        else:
            if self.thread is not None and self.thread.isRunning():
                self.worker.stop()
                self.thread.quit()
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Terminating camera thread: no valid servers')
            else:
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: No valid camera servers, unable to start thread')
