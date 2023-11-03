VERSION = "v0.09"
import sys, time, serial
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtCore


WIN_WIDTH, WIN_HEIGHT = 684, 400    # Window size
SER_TIMEOUT = 0.1                   # Timeout for serial Rx
RETURN_CHAR = "\n"                  # Char to be sent when Enter key pressed
PASTE_CHAR  = "\x16"                # Ctrl code for clipboard paste
baudrate    = 115200                # Default baud rate
portname    = "COM1"                # Default port name
hexmode     = False                 # Flag to enable hex display

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

        self.pushButton.clicked.connect(self.btnClick_Connect)
        self.pushButton_2.clicked.connect(self.btnClick_Disconnect)

    def btnClick_Connect(self) :
        print("버튼이 클릭되었습니다.")
        self.textBrowser.append("Append Text")

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