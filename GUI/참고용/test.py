# -*- coding: utf-8 -*-
import sys
import math
import re
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo

class SerialMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        super(SerialMonitor, self).__init__()
        self.port = QSerialPort()
        self.serialDataView = SerialDataView(self)
        self.serialSendView = SerialSendView(self)

        self.setCentralWidget( QtWidgets.QWidget(self) )
        layout = QtWidgets.QVBoxLayout( self.centralWidget() )
        layout.addWidget(self.serialDataView)
        layout.addWidget(self.serialSendView)
        layout.setContentsMargins(3, 3, 3, 3)
        self.setWindowTitle('Serial Monitor')

        ### Tool Bar ###
        self.toolBar = ToolBar(self)
        self.addToolBar(self.toolBar)

        ### Status Bar ###
        self.setStatusBar( QtWidgets.QStatusBar(self) )
        self.statusText = QtWidgets.QLabel(self)
        self.statusBar().addWidget( self.statusText )
        
        ### Signal Connect ###
        self.toolBar.portOpenButton.clicked.connect(self.portOpen)
        self.serialSendView.serialSendSignal.connect(self.sendFromPort)
        self.port.readyRead.connect(self.readFromPort)

    def portOpen(self, flag):
        if flag:
            self.port.setBaudRate( self.toolBar.baudRate() )
            self.port.setPortName( self.toolBar.portName() )
            self.port.setDataBits( self.toolBar.dataBit() )
            self.port.setParity( self.toolBar.parity() )
            self.port.setStopBits( self.toolBar.stopBit() )
            self.port.setFlowControl( self.toolBar.flowControl() )
            r = self.port.open(QtCore.QIODevice.ReadWrite)
            if not r:
                self.statusText.setText('Port open error')
                self.toolBar.portOpenButton.setChecked(False)
                self.toolBar.serialControlEnable(True)
            else:
                self.statusText.setText('Port opened')
                self.toolBar.serialControlEnable(False)
        else:
            self.port.close()
            self.statusText.setText('Port closed')
            self.toolBar.serialControlEnable(True)
        
    def readFromPort(self):
        data = self.port.readAll()
        if len(data) > 0:
            self.serialDataView.appendSerialText( QtCore.QTextStream(data).readAll(), QtGui.QColor(255, 0, 0) )

    def sendFromPort(self, text):
        self.port.write( text.encode() )
        self.serialDataView.appendSerialText( text, QtGui.QColor(0, 0, 255) )

class SerialDataView(QtWidgets.QWidget):
    def __init__(self, parent):
        super(SerialDataView, self).__init__(parent)
        self.serialData = QtWidgets.QTextEdit(self)
        self.serialData.setReadOnly(True)
        self.serialData.setFontFamily('Courier New')
        self.serialData.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.serialDataHex = QtWidgets.QTextEdit(self)
        self.serialDataHex.setReadOnly(True)
        self.serialDataHex.setFontFamily('Courier New')
        self.serialDataHex.setFixedWidth(500)
        self.serialDataHex.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.label = QtWidgets.QLabel('00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F')
        self.label.setFont( QtGui.QFont('Courier New') )
        self.label.setIndent(5)

        self.setLayout( QtWidgets.QGridLayout(self) )
        self.layout().addWidget(self.serialData,    0, 0, 2, 1)
        self.layout().addWidget(self.label,         0, 1, 1, 1)
        self.layout().addWidget(self.serialDataHex, 1, 1, 1, 1)
        self.layout().setContentsMargins(2, 2, 2, 2)
        
    def appendSerialText(self, appendText, color):
        self.serialData.moveCursor(QtGui.QTextCursor.End)
        self.serialData.setFontFamily('Courier New')
        self.serialData.setTextColor(color)
        self.serialDataHex.moveCursor(QtGui.QTextCursor.End)
        self.serialDataHex.setFontFamily('Courier New')
        self.serialDataHex.setTextColor(color)

        self.serialData.insertPlainText(appendText)
        
        lastData = self.serialDataHex.toPlainText().split('\n')[-1]
        lastLength = math.ceil( len(lastData) / 3 )
        
        appendLists = []
        splitedByTwoChar = re.split( '(..)', appendText.encode().hex() )[1::2]
        if lastLength > 0:
            t = splitedByTwoChar[ : 16-lastLength ] + ['\n']
            appendLists.append( ' '.join(t) )
            splitedByTwoChar = splitedByTwoChar[ 16-lastLength : ]

        appendLists += [ ' '.join(splitedByTwoChar[ i*16 : (i+1)*16 ] + ['\n']) for i in range( math.ceil(len(splitedByTwoChar)/16) ) ]
        if len(appendLists[-1]) < 47:
            appendLists[-1] = appendLists[-1][:-1]

        for insertText in appendLists:
            self.serialDataHex.insertPlainText(insertText)
        
        self.serialData.moveCursor(QtGui.QTextCursor.End)
        self.serialDataHex.moveCursor(QtGui.QTextCursor.End)

class SerialSendView(QtWidgets.QWidget):

    serialSendSignal = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super(SerialSendView, self).__init__(parent)

        self.sendData = QtWidgets.QTextEdit(self)
        self.sendData.setAcceptRichText(False)
        self.sendData.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)

        self.sendButton = QtWidgets.QPushButton('Send')
        self.sendButton.clicked.connect(self.sendButtonClicked)
        self.sendButton.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        
        self.setLayout( QtWidgets.QHBoxLayout(self) )
        self.layout().addWidget(self.sendData)
        self.layout().addWidget(self.sendButton)
        self.layout().setContentsMargins(3, 3, 3, 3)

    def sendButtonClicked(self):
        self.serialSendSignal.emit( self.sendData.toPlainText() )
        self.sendData.clear()

class ToolBar(QtWidgets.QToolBar):
    def __init__(self, parent):
        super(ToolBar, self).__init__(parent)
        
        self.portOpenButton = QtWidgets.QPushButton('Open')
        self.portOpenButton.setCheckable(True)
        self.portOpenButton.setMinimumHeight(32)

        self.portNames = QtWidgets.QComboBox(self)
        self.portNames.addItems([ port.portName() for port in QSerialPortInfo().availablePorts() ])
        self.portNames.setMinimumHeight(30)

        self.baudRates = QtWidgets.QComboBox(self)
        self.baudRates.addItems([
            '110', '300', '600', '1200', '2400', '4800', '9600', '14400', '19200', '28800', 
            '31250', '38400', '51200', '56000', '57600', '76800', '115200', '128000', '230400', '256000', '921600'
        ])
        self.baudRates.setCurrentText('115200')
        self.baudRates.setMinimumHeight(30)

        self.dataBits = QtWidgets.QComboBox(self)
        self.dataBits.addItems(['5 bit', '6 bit', '7 bit', '8 bit'])
        self.dataBits.setCurrentIndex(3)
        self.dataBits.setMinimumHeight(30)

        self._parity = QtWidgets.QComboBox(self)
        self._parity.addItems(['No Parity', 'Even Parity', 'Odd Parity', 'Space Parity', 'Mark Parity'])
        self._parity.setCurrentIndex(0)
        self._parity.setMinimumHeight(30)

        self.stopBits = QtWidgets.QComboBox(self)
        self.stopBits.addItems(['One Stop', 'One And Half Stop', 'Two Stop'])
        self.stopBits.setCurrentIndex(0)
        self.stopBits.setMinimumHeight(30)

        self._flowControl = QtWidgets.QComboBox(self)
        self._flowControl.addItems(['No Flow Control', 'Hardware Control', 'Software Control'])
        self._flowControl.setCurrentIndex(0)
        self._flowControl.setMinimumHeight(30)

        self.addWidget( self.portOpenButton )
        self.addWidget( self.portNames)
        self.addWidget( self.baudRates)
        self.addWidget( self.dataBits)
        self.addWidget( self._parity)
        self.addWidget( self.stopBits)
        self.addWidget( self._flowControl)

    def serialControlEnable(self, flag):
        self.portNames.setEnabled(flag)
        self.baudRates.setEnabled(flag)
        self.dataBits.setEnabled(flag)
        self._parity.setEnabled(flag)
        self.stopBits.setEnabled(flag)
        self._flowControl.setEnabled(flag)
        
    def baudRate(self):
        return int(self.baudRates.currentText())

    def portName(self):
        return self.portNames.currentText()

    def dataBit(self):
        return int(self.dataBits.currentIndex() + 5)

    def parity(self):
        return self._parity.currentIndex()

    def stopBit(self):
        return self.stopBits.currentIndex()

    def flowControl(self):
        return self._flowControl.currentIndex()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = SerialMonitor()
    window.show()
    app.exec()