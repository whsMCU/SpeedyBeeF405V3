# This is a Qt (PySide) terminal program


from PySide6.QtCore import QIODevice
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtWidgets import QApplication, QMainWindow
import sys
import datetime
from comport import ComPort
from controls import *

###############################################################

term_title = 'QtTerminal'

nmax = 300  # max number of COM port, used in get_free_ports()

com_port_name = ''
default_baud_rate = '115200'

window_min_height = 600
window_min_width = 900

term_min_width = 400

sms = 'SMS test'

cmd_end = b'\r'
sms_end = b'\x1A'

time_stamp = 1  # to add time stamp (1) or not (0)
echo = 0  # show echo (1) or not (0)
new_line = 1  # set add_time = 1 after receiving '\r' or '\n'

# Names of notebook's tables
tab1Name = 'Basic'
tab2Name = 'Net'
tab3Name = 'TCP/IP'
tab4Name = 'SMS'
tab5Name = 'Edit'

###############################################################

# Tab button [0,1,2,3]:
# 0 - label of the button
# 1 - command to send
# 2 - foreground color of the button
# 3 - background color of the button

# --------------- TAB1 BUTTONS ---------------
T1_0 = [['AT', 'AT', '', 'light blue'],
        ['Get info', 'ATI', '', 'light blue'],
        ['Get FW version', 'AT+CGMR', '', 'light blue'],
        ['Get IMEI', 'AT+CGSN', '', 'light blue'],
        ['Get IMSI', 'AT+CIMI', '', 'light blue'],
        ['Get ICCID', 'AT+CCID', '', 'light blue'],
        ['', '', '', ''],
        ['Get func mode', 'AT+CFUN?', '', 'yellow'],
        ['Set minimum mode [0]', 'AT+CFUN=0', '', 'yellow'],
        ['Set full mode [1]', 'AT+CFUN=1', '', 'yellow'],
        ['Set airplane mode [4]', 'AT+CFUN=4', '', 'yellow'],
        ['', '', '', ''],
        ['Enable echo', 'ATE1', '', ''],
        ['Disable echo', 'ATE0', '', ''],
        ['', '', '', '']]

T1_1 = [['Get baud rate', 'AT+IPR?', '', '#33CCCC'],
        ['Set 9600 bps', 'AT+IPR=9600', '', '#33CCCC'],
        ['Set 115200 bps', 'AT+IPR=115200', '', '#33CCCC'],
        ['Set auto baud rate', 'AT+IPR=0', '', '#33CCCC'],
        ['', '', '', ''],
        ['Get error ind mode', 'AT+CMEE?', '', ''],
        ['Set error min mode [0]', 'AT+CMEE=0', '', ''],
        ['Set error code mode [1]', 'AT+CMEE=1', '', ''],
        ['Set error verbose mode [2]', 'AT+CMEE=2', '', ''],
        ['', '', '', ''],
        ['AT+CPIN?', 'AT+CPIN?', '', '#77dd77'],
        ['', '', '', '']]

T1_2 = [['AT+CSQ', 'AT+CSQ', '', 'light blue'],
        ['', '', '', ''],
        ['AT+CREG?', 'AT+CREG?', '', '#FFB273'],
        ['AT+CREG=2', 'AT+CREG=2', '', '#FFB273'],
        ['AT+CREG=0', 'AT+CREG=0', '', '#FFB273'],
        ['', '', '', ''],
        ['AT+CGREG?', 'AT+CGREG?', '', 'light blue'],
        ['AT+CGREG=2', 'AT+CGREG=2', '', ''],
        ['AT+CGREG=0', 'AT+CGREG=0', '', ''],
        ['', '', '', ''],
        ['AT+CEREG?', 'AT+CEREG?', '', 'light blue'],
        ['AT+CEREG=2', 'AT+CEREG=2', '', ''],
        ['AT+CEREG=5', 'AT+CEREG=5', '', ''],
        ['AT+CEREG=0', 'AT+CEREG=0', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T1 = [T1_0, T1_1, T1_2]

# --------------- TAB2 BUTTONS ---------------
T2_0 = [['Get RSSI', 'AT+CSQ', '', ''],
        ['', '', '', ''],
        ['AT+CREG?', 'AT+CREG?', '', ''],
        ['AT+CREG=0', 'AT+CREG=0', '', ''],
        ['AT+CREG=1', 'AT+CREG=1', '', ''],
        ['AT+CREG=2', 'AT+CREG=2', '', ''],
        ['', '', '', ''],
        ['AT+CGREG?', 'AT+CGREG?', '', '#FFB299'],
        ['AT+CGREG=0', 'AT+CGREG=0', '', '#FFB299'],
        ['AT+CGREG=1', 'AT+CGREG=1', '', '#FFB299'],
        ['AT+CGREG=2', 'AT+CGREG=2', '', '#FFB299'],
        ['', '', '', ''],
        ['AT+CEREG?', 'AT+CEREG?', '', ''],
        ['AT+CEREG=0', 'AT+CEREG=0', '', ''],
        ['AT+CEREG=1', 'AT+CEREG=1', '', ''],
        ['AT+CEREG=2', 'AT+CEREG=2', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T2_1 = [['Get APN', 'AT+NETAPN?', '', ''],
        ['Set APN', 'AT+NETAPN="IP","",""', '', ''],
        ['', '', '', ''],
        ['Get PDP context', 'AT+CGDCONT?', '', 'yellow'],
        ['Set PDP context', 'AT+CGDCONT=1,"IP"', '', 'yellow'],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T2_2 = [['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T2 = [T2_0, T2_1, T2_2]

# --------------- TAB3 BUTTONS ---------------
T3_0 = [['Get PPP state', 'AT+XIIC?', '', ''],
        ['Set PPP link', 'AT+XIIC=1', '', '#77dd77'],
        ['Close PPP link', 'AT+XIIC=0', '', 'pink'],
        ['', '', '', ''],
        ['Set TCP connection', 'AT+TCPSETUP=0,google.com,80', '', ''],
        ['Request to send data', 'AT+TCPSEND=0,10', '', ''],
        ['Send data', 'HEAD  \r\n\r\n', '', ''],
        ['', '', '', ''],
        ['Read 100 bytes', 'AT+TCPREAD=0,100', '', ''],
        ['Get TCP connection state', 'AT+IPSTATUS=0', '', ''],
        ['Close TCP connection', 'AT+TCPCLOSE=0', '', 'pink'],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T3_1 = [['Get receive mode', 'AT+RECVMODE?', '', ''],
        ['Send to UART, ASCII [1,0]', 'AT+RECVMODE=1,0', '', ''],
        ['Send to UART, HEX [1,1]', 'AT+RECVMODE=1,1', '', ''],
        ['Save in buff, ASCII [0,0]', 'AT+RECVMODE=0,0', '', ''],
        ['Save in buff, HEX [0,1]', 'AT+RECVMODE=0,1', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T3_2 = [['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T3 = [T3_0, T3_1, T3_2]

# --------------- TAB4 BUTTONS ---------------
T4_0 = [['Get SMS mode', 'AT+CMGF?', '', ''],
        [' Set Text SMS mode', 'AT+CMGF=1', '', ''],
        ['', '', '', ''],
        ['Get char set', 'AT+CSCS?', '', ''],
        ['Set GSM char mode', 'AT+CSCS="GSM"', '', ''],
        ['Set IRA char mode', 'AT+CSCS="IRA"', '', ''],
        ['Set UCS2 char mode', 'AT+CSCS="UCS2"', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T4_1 = [['Get SMS indication mode', 'AT+CNMI?', '', ''],
        ['Set SMS ind 22000 mode', 'AT+CNMI=2,2,0,0,0', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T4_2 = [['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', ''],
        ['', '', '', '']]

T4 = [T4_0, T4_1, T4_2]

# --------------- TAB5 BUTTONS ---------------
T5 = ['AT+CLAC', 'AT+RDAVER', 'AT+VER?', 'AT+SETBND?', 'AT+UPTIME', 'AT+IMEI', 'AT+VBATVOL', 'AT+NVLOSSCOV']


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(term_title)
        self.statusBar().showMessage('Welcome!')
        # central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        hbox = QHBoxLayout(central_widget)
        # create term
        self.term = QTextEdit()
        self.term.setReadOnly(True)
        self.term.setMinimumWidth(term_min_width)
        self.term.setStyleSheet("""
                background-color: #000000;
                color: #FFFFFF;
                font-family: Titillium;
                font-size: 12px;
                """)
        hbox.addWidget(self.term)
        # create vbox (vertical layout)
        vbox = QVBoxLayout()
        # add vbox to main window
        hbox.addLayout(vbox)
        # create Controls and bind handlers
        self.controls = Controls()
        self.controls.clear_btn.clicked.connect(self.clear_term)
        self.controls.copy_btn.clicked.connect(self.copy)
        self.controls.cut_btn.clicked.connect(self.cut)
        self.controls.info_btn.clicked.connect(self.ports_info)
        self.controls.free_btn.clicked.connect(self.get_free_ports)
        self.controls.time_box.toggled.connect(self.time_box_toggled)
        self.controls.time_box.setChecked(True)
        self.controls.echo_box.toggled.connect(self.echo_box_toggled)
        # create Com port
        self.port = ComPort(com_port_name, default_baud_rate)
        self.port.ser.readyRead.connect(self.on_port_rx)
        self.port.ser.errorOccurred.connect(self.port_error)
        # create any_panels and bind handlers
        self.any_panel_1 = SendAny()
        self.any_panel_2 = SendAny()
        self.any_panel_3 = SendAny()
        self.any_panel_1.any_btn.clicked.connect(self.send_any)
        self.any_panel_2.any_btn.clicked.connect(self.send_any)
        self.any_panel_3.any_btn.clicked.connect(self.send_any)
        # create notebook
        self.notebook = Notebook()
        # add tables to the notebook
        self.notebook.add_tab_btn(tab1Name, T1, self.send)
        self.notebook.add_tab_btn(tab2Name, T2, self.send)
        self.notebook.add_tab_btn(tab3Name, T3, self.send)
        self.notebook.add_tab_btn(tab4Name, T4, self.send)
        self.notebook.add_tab_edit(tab5Name, 10, T5, self.send_any)
        # add controls to side panel
        vbox.addWidget(self.port)
        vbox.addWidget(self.controls)
        vbox.addWidget(self.any_panel_1)
        vbox.addWidget(self.any_panel_2)
        vbox.addWidget(self.any_panel_3)
        vbox.addWidget(self.notebook)

    def send(self, btn):
        global cmd_end
        global sms_end
        cmd_to_send = btn.get_cmd()
        if cmd_to_send:
            if cmd_to_send == sms:  # if need to send SMS
                self.write(cmd_to_send.encode('ascii') + sms_end)
            else:
                if isinstance(cmd_to_send, str):  # if type of cmd_to_send is a <string>
                    self.write(cmd_to_send.encode('ascii') + cmd_end)
                else:
                    print('send(): something went wrong sending to UART (ASCII encoding failed)!')

    def send_any(self):
        global cmd_end
        if self.port.com_opened:
            ref = self.sender()  # get object created received signal
            cmd_to_send = ref.parent().any_field.text()  # get text from any_field using parent
            if cmd_to_send:
                self.write(cmd_to_send.encode('ascii') + cmd_end)

    def ports_info(self):
        ser_info = QSerialPortInfo()
        ports = ser_info.availablePorts()
        self.term.insertPlainText('\r' + '-' * 18 + ' Ports info ' + '-' * 18 + '\r')
        for ser in ports:
            self.term.insertPlainText(ser.portName() + '\r')
            self.term.insertPlainText(ser.description() + '\r')
            if ser.hasVendorIdentifier():
                self.term.insertPlainText('VID: ' + hex(ser.vendorIdentifier()) + '\r')
            if ser.hasProductIdentifier():
                self.term.insertPlainText('PID: ' + hex(ser.productIdentifier()) + '\r')
            self.term.insertPlainText('Manufacturer: ' + ser.manufacturer() + '\r')
            self.term.insertPlainText('-' * 50 + '\r')
        self.term.ensureCursorVisible()

    def get_free_ports(self):
        n = 0
        self.term.insertPlainText('\r' + '-' * 17 + ' Free ports ' + '-' * 17 + '\r')
        temp_port = QSerialPort()
        while n < nmax:
            n = n + 1
            temp_port.setPortName('COM' + str(n))
            res = temp_port.open(QIODevice.ReadWrite)
            if res:  # port was successfully opened
                self.term.insertPlainText('COM' + str(n) + '\r')
                temp_port.close()
        self.term.insertPlainText('-' * 50 + '\r')
        self.term.ensureCursorVisible()

    def write(self, data):
        global echo
        if echo:
            str_data = self.decode_and_prepare(b'    <<<    ' + data)
            self.term.insertPlainText(str_data)
            self.term.ensureCursorVisible()
        self.port.write(data)

    def clear_term(self):
        try:
            self.term.clear()  # clear terminal
            self.statusBar().showMessage('Terminal cleared')
        except Exception:
            self.statusBar().showMessage('Failed to clear terminal!')

    def copy(self):
        try:
            curs = self.term.textCursor()  # cursor, used when unselecting all
            self.term.selectAll()  # select all
            self.term.copy()  # copy to the clipboard
            self.term.setTextCursor(curs)  # unselect all
            self.statusBar().showMessage('Successfully copied from terminal')
        except Exception:
            self.statusBar().showMessage('Failed to copy from terminal!')

    def cut(self):
        try:
            self.term.selectAll()  # select all
            self.term.copy()  # copy to the clipboard
            self.term.clear()  # clear terminal
            self.statusBar().showMessage('Successfully cut from terminal')
        except Exception:
            self.statusBar().showMessage('Failed to cut from terminal!')

    def closeEvent(self, event):
        self.port.close_port()
        event.accept()

    @staticmethod
    def nice_hex(binstr):  # convert binary string to readable HEX-like string: F1 34 0A 5D 00 7A...
        nicestr = ''
        for x in binstr:
            nicestr = nicestr + bytes([x]).hex().upper() + ' '
        return nicestr[:-1]

    @staticmethod
    def show_hex(ch):  # convert byte to readable HEX-like symbol like: <0x1A>
        return '<0x' + ch.hex().upper() + '>'

    def decode_and_format(self, binstr):  # types of binstr: <class 'PySide6.QtCore.QByteArray'>, <class 'bytes'>
        global time_stamp
        global new_line
        ascii_str = ''
        # for i in range(0, binstr.size()):
        for i in binstr:
            if isinstance(i, int):  # if type of binstr is <bytes> then type of i will be <int>!
                i = bytes([i])  # convert <int> to <bytes>
            if i == b'\x00':  # when self.term.insertPlainText('\x00') we get Exception: ValueError: embedded null character
                ascii_symbol = '<0x00>'  # replace byte 0x00 with string '<0x00>'
            else:
                try:
                    ascii_symbol = i.decode("ascii")
                except Exception:
                    ascii_symbol = self.show_hex(i)
            if ascii_symbol == '\r' or ascii_symbol == '\n':
                new_line = 1
                ascii_symbol = ''  # remove '\r' and '\n' to not to print empty lines
            else:
                if new_line:
                    ascii_str = ascii_str + '\r'
                    new_line = 0
                    if time_stamp:
                        curr_time = datetime.datetime.now()
                        ascii_str = ascii_str + curr_time.strftime('%H:%M:%S:%f')[:12] + '    '
            ascii_str = ascii_str + ascii_symbol
        return ascii_str

    def on_port_rx(self):
        num_rx_bytes = self.port.ser.bytesAvailable()
        rx_bytes = self.port.ser.read(num_rx_bytes)
        data = self.decode_and_format(rx_bytes)
        try:
            self.term.insertPlainText(data)
            self.term.ensureCursorVisible()
        except Exception:
            self.statusBar().showMessage('on_port_rx: ERROR')

    def echo_box_toggled(self, checked):
        global echo
        if checked:
            echo = 1
        else:
            echo = 0

    def time_box_toggled(self, checked):
        global time_stamp
        if checked:
            time_stamp = 1
        else:
            time_stamp = 0

    def indicate_port_error(self,
                            err):  # separated function for indication is a good idea if we have more than one serial port
        if err == QSerialPort.SerialPortError.NoError:
            err_msg = 'OK'
        elif err == QSerialPort.SerialPortError.DeviceNotFoundError:
            err_msg = 'device not found'
        elif err == QSerialPort.SerialPortError.PermissionError:
            err_msg = 'permission error'
        elif err == QSerialPort.SerialPortError.OpenError:
            err_msg = 'open error'
        elif err == QSerialPort.SerialPortError.NotOpenError:
            err_msg = 'not open error'
        elif err == QSerialPort.SerialPortError.WriteError:
            err_msg = 'write error'
        elif err == QSerialPort.SerialPortError.ReadError:
            err_msg = 'read error'
        elif err == QSerialPort.SerialPortError.ResourceError:
            err_msg = 'resource error'
        elif err == QSerialPort.SerialPortError.UnsupportedOperationError:
            err_msg = 'unsupported operation'
        elif err == QSerialPort.SerialPortError.TimeoutError:
            err_msg = 'timeout error'
        elif err == QSerialPort.SerialPortError.UnknownError:
            err_msg = 'unknown error'
        else:
            err_msg = 'undefined error'
        self.statusBar().showMessage('Serial port: ' + err_msg)

    def port_error(self):
        self.indicate_port_error(self.port.ser.error())


def main():
    app = QApplication([])
    main_win = MainWindow()
    main_win.resize(window_min_width, window_min_height)
    main_win.show()
    sys.exit(app.exec())  # PySide6
    # sys.exit(app.exec_())        # PySide2


if __name__ == '__main__':
    main()
