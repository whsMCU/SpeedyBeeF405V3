# Controls for terminals

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QCheckBox
from PySide6.QtWidgets import QTabWidget, QTextEdit, QGridLayout, QButtonGroup, QLineEdit, QGroupBox
        

class Controls(QGroupBox):
    def __init__(self):
        super().__init__()
        self.setTitle('Controls')
        # add layout
        layout = QGridLayout(self)
        # create buttons, etc
        self.time_box = QCheckBox('Add time', self)
        layout.addWidget(self.time_box, 0, 0)
        #
        self.echo_box = QCheckBox('Echo Tx', self)
        layout.addWidget(self.echo_box, 0, 1)
        #
        self.clear_btn = QPushButton('Clear term')
        self.clear_btn.setStyleSheet('background-color: #333333; color: #eeeeee;')
        layout.addWidget(self.clear_btn, 1, 0)
        #
        self.copy_btn = QPushButton('Copy from term')
        self.copy_btn.setStyleSheet('background-color: #999999;')
        layout.addWidget(self.copy_btn, 0, 2)
        #
        self.cut_btn = QPushButton('Cut from term')
        self.cut_btn.setStyleSheet('background-color: #999999;')
        layout.addWidget(self.cut_btn, 1, 2)
        #
        self.info_btn = QPushButton('Ports info')
        self.info_btn.setStyleSheet('background-color: #ffff00;')
        layout.addWidget(self.info_btn, 0, 3)
        #
        self.free_btn = QPushButton('Get free ports')
        self.free_btn.setStyleSheet('background-color: #ffff00;')
        layout.addWidget(self.free_btn, 1, 3)


class ControlsTCP(QGroupBox):
    def __init__(self):
        super().__init__()
        self.setTitle('Controls')
        # add layout
        layout = QGridLayout(self)
        # Line 0
        # create clear_btn
        self.clear_btn = QPushButton('Clear term')
        self.clear_btn.setStyleSheet('background-color: #333333; color: #eeeeee;')
        layout.addWidget(self.clear_btn, 0, 0)


class SendAny(QWidget):
    def __init__(self):
        super().__init__()
        # add layout
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        #
        self.any_field = QLineEdit()
        self.any_field.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.any_field)
        #
        self.any_btn = QPushButton('Send')
        self.any_btn.setStyleSheet('background-color: #ffffff;')
        layout.addWidget(self.any_btn)


class NewButton(QPushButton):
    cmd = ''

    def __init__(self, lbl):
        super().__init__()
        self.setText(lbl)
        self.setMinimumHeight(20)

    def set_cmd(self, cmd):
        self.cmd = cmd

    def get_cmd(self):
        return self.cmd


class Notebook(QTabWidget):

    def_btn_fg_color = 'black'
    def_btn_bg_color = '#eeeeee'
    btn_font_family = 'Titillium'
    btn_font_size = '12px'

    def __init__(self):
        super().__init__()

    # method to add Tables with buttons
    def add_tab_btn(self, tab_name, btn_data, handler):
        # create new table
        tab = QWidget()
        self.addTab(tab, tab_name)
        layout = QGridLayout(tab)
        # add buttons' group
        group = QButtonGroup(self)
        group.buttonClicked.connect(handler)
        c = 0        # defines number of column of buttons in the grid
        for col in btn_data:
            i = 0        # counter to define number of line of the buttons in the grid
            for btn in col:
                if btn[0]:                         # if the button's label is not empty
                    b = NewButton(btn[0])          # create button object
                    b.set_cmd(btn[1])              # set button's command
                    group.addButton(b)
                    layout.addWidget(b, i, c)
                    if not btn[2]: btn[2] = self.def_btn_fg_color   # if foreground colour is not defined use default one
                    if not btn[3]: btn[3] = self.def_btn_bg_color   # if background colour is not defined use default one
                    b.setStyleSheet('background-color: ' + btn[3] + '; ' +
                                    'color: ' + btn[2] + '; ' +
                                    'font-family: ' + self.btn_font_family + '; ' +
                                    'font-size: ' + self.btn_font_size + ';')
                i = i + 1   # next line
            c = c + 1       # next column

    # method to add Tables with editable fields
    def add_tab_edit(self, tab_name, num_of_fields, tab_data, handler):
        # create new table
        tab = QWidget()
        self.addTab(tab, tab_name)
        layout = QVBoxLayout(tab)
        for each in range(num_of_fields):
            fld = SendAny()
            fld.any_btn.clicked.connect(handler)
            fld.any_btn.setStyleSheet('background-color: #FFB273')
            layout.addWidget(fld)
            try:
                fld.any_field.setText(tab_data[each])
            except Exception:
                pass


class LogMonitor(QGroupBox):
    def __init__(self):
        super().__init__()
        self.setTitle('Log monitor')
        # add layout
        layout = QVBoxLayout(self)
        # create log monitor
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet("""
                        background-color: #101010;
                        color: #FFFFFF;
                        font-family: Ariel;
                        font-size: 11px;
                        """)
        layout.addWidget(self.log)
        # create clear_log_btn
        self.clear_log_btn = QPushButton('Clear log')
        self.clear_log_btn.setStyleSheet('background-color: #333333; color: #eeeeee;')
        self.clear_log_btn.clicked.connect(self.clear_log)
        layout.addWidget(self.clear_log_btn)

    def clear_log(self):
        self.log.clear()               # clear log monitor
