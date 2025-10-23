# This file contains the NTTreeManager class, which is responsible for managing the NetworkTables tree view for tuning.

import time
from datetime import datetime
from PyQt6 import QtWidgets
from ntcore import NetworkTableType

class NTTreeManager:
    def __init__(self, ui):
        self.ui = ui

    def toggle_network_tables(self):
        """Shows or hides the NetworkTables tree widget."""
        if self.ui.qt_tree_widget_nt.isHidden():
            self.refresh_tree()
            self.ui.qt_tree_widget_nt.show()
        else:
            self.ui.qt_tree_widget_nt.hide()

    def refresh_tree(self):
        """Reads networktables and updates tree and combo widgets."""
        if self.ui.nt_manager.isConnected():
            self.ui.nt_manager.report_nt_status()
            self.ui.qt_tree_widget_nt.clear()
            entries = self.ui.nt_manager.getEntries('/', 0)
            self.ui.sorted_tree = sorted([e.getName() for e in entries])

            self.filter_nt_keys_combo()

            nt_dict = {}
            levels = [s[1:].split('/') for s in self.ui.sorted_tree]
            for path in levels:
                current_level = nt_dict
                for part in path:
                    if part not in current_level:
                        current_level[part] = {}
                    current_level = current_level[part]

            self.ui.qlistwidget_commands.clear()
            self.ui.command_dict.clear() # Clear old commands
            for item in self.ui.sorted_tree:
                if 'running' in item:
                    command_name = item.split('/')[2]
                    self.ui.qlistwidget_commands.addItem(command_name)
                    self.ui.command_dict.update({command_name: {'nt_topic': item, 'nt_entry': self.ui.nt_manager.getEntry(item)}})

                entry_value = self.ui.nt_manager.getEntry(item).getValue()
                if entry_value is not None:
                    value = entry_value.value()
                    age = int(time.time() - entry_value.last_change() / 1E6)
                    levels = item[1:].split('/')
                    if len(levels) == 2:
                        nt_dict[levels[0]][levels[1]] = value, age
                    elif len(levels) == 3:
                        nt_dict[levels[0]][levels[1]][levels[2]] = value, age
                    elif len(levels) == 4:
                        nt_dict[levels[0]][levels[1]][levels[2]][levels[3]] = value, age

            self.fill_item(self.ui.qt_tree_widget_nt.invisibleRootItem(), nt_dict)
            self.ui.qt_tree_widget_nt.resizeColumnToContents(0)
            self.ui.qt_tree_widget_nt.setColumnWidth(1, 100)
        else:
            self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Unable to connect to server')

    def filter_nt_keys_combo(self):
        """Used to simplify the nt key list combo box entries."""
        if self.ui.sorted_tree is not None:
            self.ui.qcombobox_nt_keys.clear()
            filter_text = self.ui.qt_text_entry_filter.toPlainText()
            filtered_keys = [key for key in self.ui.sorted_tree if filter_text in key]
            self.ui.qcombobox_nt_keys.addItems(filtered_keys)

    def update_key(self):
        """Used to modify an nt key value from the TUNING tab."""
        key = self.ui.qcombobox_nt_keys.currentText()
        nt_entry = self.ui.nt_manager.getEntry(key)
        entry_type = nt_entry.getType()
        new_val_str = self.ui.qt_text_new_value.toPlainText()
        print(f'Update key was called on {key}, which is a {entry_type}. Setting it to {new_val_str}', flush=True)
        try:
            if entry_type == NetworkTableType.kDouble:
                nt_entry.setDouble(float(new_val_str))
            elif entry_type == NetworkTableType.kString:
                nt_entry.setString(new_val_str)
            elif entry_type == NetworkTableType.kBoolean:
                nt_entry.setBoolean(eval(new_val_str))
            else:
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: {key} type {entry_type} not in [double, bool, string]')
        except Exception as e:
            self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Error occurred in setting {key} - {e}')
        self.ui.qt_text_new_value.clear()
        self.refresh_tree()

    def update_selected_key(self):
        """Updates the display with the current value of the selected NT key."""
        key = self.ui.qcombobox_nt_keys.currentText()
        nt_entry = self.ui.nt_manager.getEntry(key)
        value = nt_entry.getValue()
        if value is not None:
            self.ui.qt_text_current_value.setPlainText(str(value.value()))

    def qt_tree_widget_nt_clicked(self, item):
        """Sends the clicked item from the tree to the filter for the nt selection combo box."""
        self.ui.qt_text_entry_filter.clear()
        self.ui.qt_text_entry_filter.setPlainText(item.data())

    def command_list_clicked(self, item):
        """Shortcut where we click the command list, fire off (or end) the command."""
        cell_content = item.data()
        command_nt_entry = self.ui.command_dict.get(cell_content, {}).get('nt_entry')
        if command_nt_entry:
            toggled_state = not command_nt_entry.getBoolean(True)
            print(f'You clicked {cell_content}. Firing command...', flush=True)
            command_nt_entry.setBoolean(toggled_state)

    def depth(self, d):
        if isinstance(d, dict):
            return 1 + (max(map(self.depth, d.values())) if d else 0)
        return 0

    def fill_item(self, widget, value):
        """Helper function for filling the NT tree widget."""
        if value is None:
            return
        elif isinstance(value, dict) and self.depth(value) > 1:
            for key, val in sorted(value.items()):
                self.new_item(parent=widget, text=str(key), val=val)
        elif isinstance(value, dict):
            for key, val in sorted(value.items()):
                child = QtWidgets.QTreeWidgetItem([str(key), str(val[0]), str(val[1])])
                self.fill_item(child, val)
                widget.addChild(child)
        else:
            pass

    def new_item(self, parent, text, val=None):
        """Helper function for filling the NT tree widget."""
        if val is None:
            child = QtWidgets.QTreeWidgetItem([text, 'noval'])
        else:
            if isinstance(val, dict):
                child = QtWidgets.QTreeWidgetItem([text])
            else:
                child = QtWidgets.QTreeWidgetItem([text, str(val[0]), str(val[1])])
        self.fill_item(child, val)
        parent.addChild(child)
        child.setExpanded(True)
