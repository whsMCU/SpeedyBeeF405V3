"MultiWii: Python MSP communication Library"

import serial, struct

class MultiWii:
    def __init__(self, SerialPort:str, BaudRate:int=115200, **kwargs):
        """
        Initialise Serial communication with Flight Controller
        Takes Serial Port address(required) and Baud Rate (defaults to 115200) as parameters 
        """
        self.com = serial.Serial(SerialPort, BaudRate, timeout=1, **kwargs)

    def send(self, command, data:list=[], datatype:str=""):
        size = 2 * ((len(data)) & 0xFF)
        senddata = ['$'.encode('utf-8'), 'M'.encode('utf-8'), command['direction'].encode('utf-8'), size, command['message_id']] + data
        checksum = self.crc(command['message_id'], size, data, datatype)
        senddata.append(checksum)
        self.com.write(struct.pack('<3c2B'+datatype+'B', *senddata))
        return True

    def crc(self, id, size, data, datatype):
        crc = size ^ id
        if not len(data) == 0:
            for i in struct.pack('<'+datatype, data):
                crc ^= i
        return crc

    def receive(self):
        while True:
            header = self.com.read().decode('utf-8')
            if header == '$':
                header += self.com.read(2).decode('utf-8')
                break
        datalength = struct.unpack('<b', self.com.read())[0]
        print("datalength: "+str(datalength))
        code = struct.unpack('<b', self.com.read())
        print('code: '+str(code))
        data = self.com.read(datalength)
        received = struct.unpack('<'+'h'*int(datalength/2),data)
        self.com.flushInput()
        self.com.flushOutput()
        return received