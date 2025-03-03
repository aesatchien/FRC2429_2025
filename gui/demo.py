import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton
from PyQt6 import QtCore
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.uic.Compiler.qtproxies import QtCore


class VideoDashboard(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("MJPEG Stream Viewer with Fallback Image")
        self.setGeometry(100, 100, 800, 600)

        # Create a web view
        self.browser = QWebEngineView()

        # Set initial content (fallback image)
        self.fallback_image = "./png/blockhead_camera.png"  # Change to your image path
        self.mjpeg_stream = QtCore.QUrl("http://127.0.0.1:1186/stream.mjpg")
        self.browser.setHtml(f'<img src="{self.fallback_image}" width="100%" height="100%">')

        # Button to toggle camera
        self.toggle_button = QPushButton("Enable Camera Stream")
        self.toggle_button.clicked.connect(self.toggle_stream)

        # Set up layout
        central_widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.browser)
        layout.addWidget(self.toggle_button)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.camera_enabled = True  # Track camera state

    def toggle_stream(self):
        """Toggles between the MJPEG stream and fallback image."""
        if self.camera_enabled:
            self.browser.setHtml(f'<img src="{self.fallback_image}" width="100%" height="100%">')
            self.toggle_button.setText("Enable Camera Stream")
        else:
            self.browser.setUrl(self.mjpeg_stream)
            self.toggle_button.setText("Disable Camera Stream")
        self.camera_enabled = not self.camera_enabled

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoDashboard()
    window.show()
    sys.exit(app.exec())
