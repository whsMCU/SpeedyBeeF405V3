import sys
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QMainWindow, QPushButton
class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 타이틀바 내용 설정
        self.setWindowTitle('PyQt5')
        # 실행 위치
        self.move(300, 300)
        # 사이즈
        self.resize(400, 200)

        # 버튼 생성
        btn = QPushButton("버튼1", self)

        # 레이아웃 생성
        layout = QHBoxLayout()
        # 레이아웃에 버튼 넣기
        layout.addWidget(btn)
        # 최상위 UI에 생성한 Layout 넣기
        self.setLayout(layout)

        # 보여주기
        self.show()

if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = MyApp()
   sys.exit(app.exec_())