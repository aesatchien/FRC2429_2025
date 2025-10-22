# C:/Cory/CJH Shared/Python/FRC/2025/gui_refactor/generate_type_hints.py
import xml.etree.ElementTree as ET
from pathlib import Path

# --- Configuration ---
UI_FILE = 'layout_2025.ui'
# Mapping of UI class names to their PyQt6 modules.
# Add to this dictionary if you use other widget types.
CLASS_MAP = {
    'QMainWindow': 'QtWidgets.QMainWindow',
    'QWidget': 'QtWidgets.QWidget',
    'QPlainTextEdit': 'QtWidgets.QPlainTextEdit',
    'QTabWidget': 'QtWidgets.QTabWidget',
    'QGroupBox': 'QtWidgets.QGroupBox',
    'QLabel': 'QtWidgets.QLabel',
    'QLCDNumber': 'QtWidgets.QLCDNumber',
    'QComboBox': 'QtWidgets.QComboBox',
    'QListWidget': 'QtWidgets.QListWidget',
    'QPushButton': 'QtWidgets.QPushButton',
    'QTreeWidget': 'QtWidgets.QTreeWidget',
    'QRadioButton': 'QtWidgets.QRadioButton',
    'QMenuBar': 'QtWidgets.QMenuBar',
    'QMenu': 'QtWidgets.QMenu',
    'QStatusBar': 'QtWidgets.QStatusBar',
    'QAction': 'QtGui.QAction',
    # Custom Widgets - map them to their imported class name
    'QLabel2': 'qlabel2.QLabel2',
    'WarningLabel': 'warning_label.WarningLabel',
}


# --- End Configuration ---

def generate_hints(ui_path: Path):
    """Parses a .ui file and prints type hints for all named widgets."""
    if not ui_path.exists():
        print(f"Error: UI file not found at '{ui_path}'")
        return

    print("# --- IDE Type Hinting (auto-generated) ---")
    print("# The following attributes are dynamically added by uic.loadUi and are not visible to IDEs.")
    print("# This section provides type hints to enable autocompletion and static analysis.")

    tree = ET.parse(ui_path)
    root = tree.getroot()

    # Find all <widget> and <action> tags that have a 'name' attribute
    widgets = root.findall(".//*[@name]")

    for widget in sorted(widgets, key=lambda w: w.attrib['name']):
        widget_name = widget.attrib['name']

        # Determine the class of the widget/element
        if widget.tag == 'action':
            widget_class = 'QAction'
        elif 'class' in widget.attrib:
            widget_class = widget.attrib['class']
        else:
            # Skip elements like <menu> that have a name but no class
            continue

        if widget_class in CLASS_MAP:
            qt_class = CLASS_MAP[widget_class]
            print(f"self.{widget_name}: {qt_class}")
        else:
            print(f"# WARNING: No type hint mapping for class '{widget_class}' (widget: {widget_name})")

    print("# --- End IDE Type Hinting ---")


if __name__ == '__main__':
    ui_file_path = Path(__file__).parent / UI_FILE
    generate_hints(ui_file_path)
