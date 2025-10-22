import sys
from PyQt6 import QtCore, QtWidgets
from dashboard_2025 import Ui

if __name__ == "__main__":
    # Attempt to set high DPI scaling if it is detected
    if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)
    
    # Compensate for DPI scaling
    screen = app.screens()[0]
    dpi_logical = int(screen.logicalDotsPerInchX())
    if dpi_logical > 96:
        print(f"We're on a scaled screen: logical dpi is {dpi_logical}")
    else:
        print(f"We're not on a scaled screen: logical dpi is {dpi_logical}")

    ui = Ui()

    try:
        sys.exit(app.exec())
    except SystemExit:
        print('Closing application.')
