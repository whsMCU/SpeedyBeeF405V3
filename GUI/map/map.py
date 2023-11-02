from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton, QLabel, QFrame, QComboBox
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt
import requests
import sys
 
QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
 
class myWidget(QWidget):
 
    def __init__(self):
        super().__init__()
        self.setFixedSize(640,640)
         
        label = QLabel('위경도좌표(xx.xxxxxx,xx.xxxxxx) or 도시명(Busan)')
        self.lineEdit = QLineEdit(self)
        self.cmb = QComboBox(self)
        txt = [str(i) for i in range(1,21)]
        self.cmb.addItems(txt)
        self.cmb.setCurrentIndex(15)
        self.btn = QPushButton('구글 맵 그리기', self)
 
        hbox = QHBoxLayout()
        hbox.addWidget(label)
        hbox.addWidget(self.lineEdit)
        hbox.addWidget(self.cmb)
        hbox.addWidget(self.btn)
 
        self.img = QLabel('', self)
        self.img.setFrameStyle(QFrame.Box)
 
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.img)
        self.setLayout(vbox)
 
        # signal
        self.btn.clicked.connect(self.clickBtn)
 
    def clickBtn(self):
        BASE_URL    = 'https://maps.googleapis.com/maps/api/staticmap?'
        API_KEY     = '여기에 API KEY 입력'
        ZOOM        = self.cmb.currentIndex()+1
        CITY        = self.lineEdit.text()
 
        W = self.img.width()
        H = self.img.height()        
 
        URL = (BASE_URL 
       + f'center={CITY}'
       + f'&zoom={ZOOM}'
       + f'&size={W}x{H}&scale=2'
       + '&markers=color:red%7Clabel:S%7C'+CITY
       + f'&key={API_KEY}')
 
        # HTTP request
        response = requests.get(URL)
 
        # image scaled and draw
        img = QPixmap()
        img.loadFromData(response.content)
        img = img.scaled(img.width()//2, img.height()//2, Qt.KeepAspectRatio, Qt.SmoothTransformation)
 
        self.img.setPixmap(img)
 
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = myWidget()
    w.show()
    sys.exit(app.exec_())