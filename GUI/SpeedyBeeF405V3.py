VERSION = "v0.09"
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5 import uic, QtCore

__platform__ = sys.platform

class SerialReadThread(QThread):
    """
    시리얼 연결이 성공하면 항상 데이터를 수신할 수 있어야 하므로
    스레드로 만들어야 한다.

    """
    # 사용자 정의 시그널 선언
    # 받은 데이터 그대로를 전달 해주기 위해 QByteArray 형태로 전달
    received_data = pyqtSignal(QByteArray, name="receivedData")

    def __init__(self, serial):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self._status = False
        self.mutex = QMutex()
        self.serial = serial

    def __del__(self):
        self.wait()

    def run(self):
        """
        들어온 데이터가 있다면 시그널을 발생
        :return:
        """
        while True:
            self.mutex.lock()
            if not self._status:
                self.cond.wait(self.mutex)

            buf = self.serial.readAll()
            if buf:
                self.received_data.emit(buf)
            self.usleep(1)
            self.mutex.unlock()

    def toggle_status(self):
        self._status = not self._status
        if self._status:
            self.cond.wakeAll()

    @pyqtSlot(bool, name='setStatus')
    def set_status(self, status):
        self._status = status
        if self._status:
            self.cond.wakeAll()

class SerialController(QWidget):

    received_data = pyqtSignal(QByteArray, name="receivedData")
    sent_data = pyqtSignal(str, name="sentData")

    def __init__(self):
        QWidget.__init__(self, flags=Qt.Widget)
        # 위젯 선언
        self.gb = QGroupBox(self.tr("Serial"))
        self.cb_port = QComboBox()
        self.cb_baud_rate = QComboBox()
        self.cb_data_bits = QComboBox()
        self.cb_flow_control = QComboBox()
        self.cb_parity = QComboBox()
        self.cb_stop_bits = QComboBox()

        # 시리얼 인스턴스 생성
        # 시리얼 스레드 설정 및 시작
        self.serial = QSerialPort()
        self.serial_info = QSerialPortInfo()
        self.serial_read_thread = SerialReadThread(self.serial)
        self.serial_read_thread.received_data.connect(lambda v: self.received_data.emit(v))
        self.serial_read_thread.start()

        self.setWindowTitle("Serial Controller")
        self._fill_serial_info()

    def _fill_serial_info(self):
        # 시리얼 상수 값들을 위젯에 채운다
        self.comboBox_Port.insertItems(0, self._get_available_port())

    @staticmethod
    def get_port_path():
        """
        현재플래폼에 맞게 경로 또는 지정어를 반환
        :return:
        """
        return {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]

    def _get_available_port(self):
        """
        255개의 포트를 열고 닫으면서 사용가능한 포트를 찾아서 반환
        :return:
        """
        available_port = list()
        port_path = self.get_port_path()

        for number in range(255):
            port_name = port_path + str(number)
            if not self._open(port_name):
                continue
            available_port.append(port_name)
            self.serial.close()
        return available_port

    def _open(self, port_name, baudrate=9600, data_bits=QSerialPort.Data8,
              flow_control=QSerialPort.NoFlowControl, parity=QSerialPort.NoParity, stop_bits=QSerialPort.OneStop):
        """
        인자값으로 받은 시리얼 접속 정보를 이용하여 해당 포트를 연결한다.
        :param port_name:
        :param baudrate:
        :param data_bits:
        :param flow_control:
        :param parity:
        :param stop_bits:
        :return: bool
        """
        info = QSerialPortInfo(port_name)
        self.serial.setPort(info)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(data_bits)
        self.serial.setFlowControl(flow_control)
        self.serial.setParity(parity)
        self.serial.setStopBits(stop_bits)
        return self.serial.open(QIODevice.ReadWrite)

    def connect_serial(self):
        serial_info = {
            "port_name": self.comboBox_Port.currentText(),
            "baudrate": self.BAUDRATES[self.cb_baud_rate.currentIndex()],
            "data_bits": self.DATABITS[self.cb_data_bits.currentIndex()],
            "flow_control": self.FLOWCONTROL[self.cb_flow_control.currentIndex()],
            "parity": self.PARITY[self.cb_parity.currentIndex()],
            "stop_bits": self.STOPBITS[self.cb_stop_bits.currentIndex()],
        }
        status = self._open(**serial_info)
        self.serial_read_thread.setStatus(status)
        return status

    def disconnect_serial(self):
        return self.serial.close()

    @pyqtSlot(bytes, name="writeData")
    def write_data(self, data):
        self.serial.writeData(data)

#UI파일 연결
#단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
form_class = uic.loadUiType("ui/SpeedyBeeF405V3.ui")[0]

#화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_class) :

    def __init__(self) :
        super().__init__()
        # 연결한 Ui를 준비한다.
        self.setupUi(self)
        # 화면을 보여준다.
        self.show()

        self.serial = SerialController()

        self.pushButton_Connect.clicked.connect(self.btnClick_Connect)
        self.pushButton_2.clicked.connect(self.btnClick_Disconnect)
        self.serial.received_data.connect(self.read_data)
        test_data = bytes([0x02]) + bytes("TEST DATA", "utf-8") + bytes([0x03])
        self.pushButton_Send.clicked.connect(lambda: self.serial.writeData(test_data))

        # 많이 사용하는 옵션을 미리 지정해 둔다.
        # 9600 8N1
        self.comboBox_Baud.setCurrentIndex(3)
        self.comboBox_Data.setCurrentIndex(3)

    def read_data(self, rd) :
        self.textBrowser.insertPlainText(str(rd, 'ascii', 'replace'))

    def btnClick_Connect(self) :
        print("버튼이 클릭되었습니다.")
        if self.serial.serial.isOpen():
            self.serial.disconnect_serial()
        else:
            self.serial.connect_serial()
        self.pushButton_Connect.setText({False: 'Connect', True: 'Disconnect'}[self.serial.serial.isOpen()])

    def btnClick_Disconnect(self) :
        print("버튼이 클릭되었습니다.")
        self.textBrowser.clear()

if __name__ == "__main__" :
    #QApplication : 프로그램을 실행시켜주는 클래스
    app = QApplication(sys.argv)

    #WindowClass의 인스턴스 생성
    myWindow = WindowClass()
    myWindow.setWindowTitle('PyQT Serial Terminal ' + VERSION)
    #프로그램 화면을 보여주는 코드
    myWindow.show()

    #프로그램을 이벤트루프로 진입시키는(프로그램을 작동시키는) 코드
    sys.exit(app.exec_())