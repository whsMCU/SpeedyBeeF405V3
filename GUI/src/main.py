# -*- coding: utf-8 -*-
"""
AFCOM - Serial Communication GUI Program
Usage: python main.py
"""
__author__ = "Mehmet Cagri Aksoy - github.com/mcagriaksoy"
__copyright__ = "Copyright 2023, The AFCOM Project"
__credits__ = ["Mehmet Cagri Aksoy"]
__license__ = "MIT"
__version__ = "1.0.1"
__maintainer__ = "Mehmet Cagri Aksoy"
__status__ = "Production"

# IMPORTS
import sys
import glob
import os

try:
    import serial
    import serial.tools.list_ports
    from serial import SerialException
    from PyQt6 import uic
    from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
    from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox

except ImportError as e:
    print("Import Error! Please install the required libraries: " + str(e))
    sys.exit(1)

# GLOBAL VARIABLES
PORTS = []

def get_serial_port():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

# MULTI-THREADING
class Worker(QObject):
    """ Worker Thread """
    finished = pyqtSignal()
    serial_data = pyqtSignal(str)

    @pyqtSlot()
    def __init__(self, ser):
        super(Worker, self).__init__()
        self.working = True
        self.ser = ser

    def work(self):
        """ Read data from serial port """
        while self.working:
            try:
                char = self.ser.read()
                #h = char.hex()
                h = str(char, 'ascii', 'replace')
                self.serial_data.emit(h)
            except SerialException:
                # Emit last error message before die!
                self.serial_data.emit("ERROR_SERIAL_EXCEPTION")
                self.working = False
            self.finished.emit()

#UI파일 연결
#단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
form_class = uic.loadUiType("ui/main_window.ui")[0]

class MainWindow(QMainWindow, form_class):
    """ Main Window """
    def __init__(self):
        """ Initialize Main Window """
        super().__init__()

        # 연결한 Ui를 준비한다.
        self.setupUi(self)
        # 화면을 보여준다.
        self.show()

        PORTS = get_serial_port()

        self.thread = None
        self.worker = None
        self.ser = None

        self.start_button.clicked.connect(self.start_loop)
        self.comboBox_Port.addItems(PORTS)
        self.comboBox_Bit.setCurrentIndex(3)

    def print_message_on_screen(self, text):
        """ Print the message on the screen """
        msg = QMessageBox()
        msg.setWindowTitle("Warning!")
        msg.setIcon(QMessageBox.Icon.Warning)
        msg.setText(text)
        msg.exec()

    def establish_serial_communication(self):
        """ Establish serial communication """
        port = self.comboBox_Port.currentText()
        baudrate = self.comboBox_Baud.currentText()
        timeout = self.comboBox_TimeOut.currentText()
        length = self.comboBox_Bit.currentText()
        parity = self.comboBox_Parity.currentText()
        stopbits = self.comboBox_StopBit.currentText()
        self.ser = serial.Serial(port=str(port),
                                    baudrate=int(baudrate, base=10),
                                    timeout=float(timeout),
                                    bytesize=int(length, base=10),
                                    parity = parity[0], #get first character
                                    stopbits = float(stopbits)
                                    )
        print("start ser : ", self.ser)
        print("port : " + str(port))
        print("baudrate : ", int(baudrate, base=10))
        print("timeout : ", float(timeout))
        print("bytesize : ", int(length, base=10))
        print("parity : ", parity[0])
        print("stopbits : ", float(stopbits))
        print(self.ser.isOpen())

    def start_loop(self):
        """ Start the loop """
        try:
            self.establish_serial_communication()
        except SerialException:
            self.print_message_on_screen(
                "Exception occured while trying establish serial communication!")

        try:
            self.worker = Worker(self.ser)   # a new worker to perform those tasks
            self.thread = QThread()  # a new thread to run our background tasks in
            # move the worker into the thread, do this first before connecting the signals
            self.worker.moveToThread(self.thread)
            # begin our worker object's loop when the thread starts running
            self.thread.started.connect(self.worker.work)
            self.worker.serial_data.connect(self.read_data_from_thread)
            # stop the loop on the stop button click
            self.end_button.clicked.connect(self.stop_loop)
            # tell the thread it's time to stop running
            self.worker.finished.connect(self.thread.quit)
            # have worker mark itself for deletion
            self.worker.finished.connect(self.worker.deleteLater)
            # have thread mark itself for deletion
            self.thread.finished.connect(self.thread.deleteLater)
            self.thread.start()
        except RuntimeError:
            self.print_message_on_screen("Exception in Worker Thread!")

    def stop_loop(self):
        """ Stop the loop """
        self.worker.working = False
        self.label_PortStatus.setText("NOT CONNECTED!")
        self.label_PortStatus.setStyleSheet('color: red')
        self.textEdit.setText('Stopped!')

    def read_data_from_thread(self, serial_data):
        """ Write the result to the text edit box"""
        # self.textEdit_BOX.append("{}".format(i))
        if "ERROR_SERIAL_EXCEPTION" in serial_data:
            self.print_message_on_screen("Serial Exception! Please check the serial port")
            self.label_PortStatus.setText("NOT CONNECTED!")
            self.label_PortStatus.setStyleSheet('color: red')
        else:
            self.comboBox_TimeOut.setEnabled(False)
            self.comboBox_Baud.setEnabled(False)
            self.comboBox_Bit.setEnabled(False)
            self.comboBox_Port.setEnabled(False)
            self.comboBox_Parity.setEnabled(False)
            self.comboBox_StopBit.setEnabled(False)
            self.save_button.setEnabled(False)
            self.start_button.setEnabled(False)

            self.textEdit.setText('Data Gathering...')
            self.label_PortStatus.setText("CONNECTED!")
            self.label_PortStatus.setStyleSheet('color: green')
            self.textEdit_BOX.insertPlainText("{}".format(serial_data))
            #textCursor = self.textEdit_BOX.textCursor()
            #textCursor.movePosition(textCursor.End)
            #self.textEdit_BOX.setTextCursor(textCursor)

    # Save the settings
    def on_save_button_clicked(self):
        """ Save the settings """
        if self.x != 0:
            self.textEdit.setText('Settings Saved!')
        else:
            self.textEdit.setText('Please enter port and speed!')

    def on_save_txt_button_clicked(self):
        """ Save the values to the TXT file"""
        with open('Output.txt', 'w', encoding='utf-8') as f:
            my_text = self.textEdit_BOX.toPlainText()
            f.write(my_text)
            f.close()

    def on_end_button_clicked(self):
        """ Stop the process """
        self.textEdit.setText('Stopped!')
        self.comboBox_TimeOut.setEnabled(True)
        self.comboBox_Baud.setEnabled(True)
        self.comboBox_Bit.setEnabled(True)
        self.comboBox_Port.setEnabled(True)
        self.comboBox_Parity.setEnabled(True)
        self.comboBox_StopBit.setEnabled(True)
        self.save_button.setEnabled(True)
        self.start_button.setEnabled(True)

    def on_send_data_button_clicked(self):
        """ Send data to serial port """
        mytext = self.textEdit_Send.toPlainText()
        print(mytext.encode())
        self.ser.write(mytext.encode())

if __name__ == "__main__":
    """ Start the UI Design """
    app = QApplication(sys.argv)
    window_object = MainWindow()

    window_object.show()
    sys.exit(app.exec())
