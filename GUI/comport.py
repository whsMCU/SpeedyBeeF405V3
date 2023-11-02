# ComPort class used in Qt terminal programs

from PySide6.QtCore import QIODevice, Qt, Slot
from PySide6.QtSerialPort import QSerialPort
from PySide6.QtWidgets import QPushButton, QComboBox, QGridLayout, QLineEdit, QLabel, QGroupBox


class ComPort(QGroupBox):  # QWidget
    def __init__(self, name='', baud_rate='115200'):
        super().__init__()
        self.setTitle('Serial port ' + name)
        # add layout
        layout = QGridLayout(self)
        # baud rates
        self.b_rates = ['9600', '14400', '19200', '28800', '33600', '38400', '57600', '115200', '230400', '460800',
                        '921600']
        self.nmax = 300  # max number of COM port
        self.port_timeout = 0.5  # COM port timeout
        # create buttons, etc
        self.info_label = QLabel('COM closed')
        self.info_label.setStyleSheet('color: red;')
        self.info_label.setMinimumWidth(100)
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label, 0, 0)
        #
        self.port_number_label = QLabel('Port number: ')
        self.port_number_label.setStyleSheet('color: #555555;')
        # self.port_number_label.setMinimumWidth(90)
        self.port_number_label.setAlignment(Qt.AlignRight)
        layout.addWidget(self.port_number_label, 0, 1)
        #
        self.port_field = QLineEdit()
        self.port_field.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.port_field, 0, 2)
        #
        self.open_btn = QPushButton('Open')
        self.open_btn.setStyleSheet('background-color: #00dd00;')
        layout.addWidget(self.open_btn, 0, 3)
        self.open_btn.clicked.connect(self.open_port)
        #
        self.baud_rate_label = QLabel('Baud rate: ')
        self.baud_rate_label.setStyleSheet('color: #555555;')
        self.baud_rate_label.setAlignment(Qt.AlignRight)
        layout.addWidget(self.baud_rate_label, 1, 1)
        #
        self.baud_rates = QComboBox()
        self.baud_rates.addItems(self.b_rates)
        if self.b_rates.count(baud_rate):
            self.baud_rates.setCurrentIndex(self.b_rates.index(baud_rate))  # set default item
        layout.addWidget(self.baud_rates, 1, 2)
        #
        self.close_btn = QPushButton('Close')
        self.close_btn.setStyleSheet('background-color: red;')
        layout.addWidget(self.close_btn, 1, 3)
        self.close_btn.clicked.connect(self.close_port)
        # serial port
        self.com_opened = 0
        # self.baud_rate = int(baud_rate)
        self.ser = QSerialPort()
        self.ser.errorOccurred.connect(self.ser_error)

    def open_port(self):
        n = 0
        if self.com_opened == 0:
            com_num = self.port_field.text()
            br = self.baud_rates.currentText()
            self.ser.setBaudRate(int(br))
            if com_num:
                if com_num.isdigit():
                    self.ser.setPortName('COM' + com_num)
                    self.ser.open(QIODevice.ReadWrite)
                    if self.ser.isOpen():
                        self.info_label.setText('COM' + com_num + ' opened')
                        self.info_label.setStyleSheet('color: #00dd00;')  # green
                        self.com_opened = 1
                    else:
                        self.info_label.setText('Can not open')
                        self.info_label.setStyleSheet('color: red;')
                        self.com_opened = 0
                else:
                    self.info_label.setText('Digits only!')
                    self.info_label.setStyleSheet('color: red;')
            else:
                while (n < self.nmax) and (self.com_opened == 0):
                    n = n + 1
                    self.ser.setPortName('COM' + str(n))
                    res = self.ser.open(QIODevice.ReadWrite)
                    if res:  # port was successfully opened
                        self.info_label.setText('COM' + str(n) + ' opened')
                        self.info_label.setStyleSheet('color: #00dd00;')  # green
                        self.com_opened = 1
                if self.com_opened == 0:
                    self.info_label.setText('Can not open')
                    self.info_label.setStyleSheet('color: red;')

    def close_port(self):
        if self.com_opened:
            self.ser.close()
            self.info_label.setText('COM closed')
            self.info_label.setStyleSheet('color: red;')
            self.com_opened = 0

    @Slot()
    def ser_error(self):
        err = self.ser.error()
        if err == QSerialPort.SerialPortError.NoError:  # if error message is "No error"
            pass
        else:
            self.close_port()  # close port if some error occurred...

    def write(self, data):
        if self.com_opened:
            self.ser.write(data)
