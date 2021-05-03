# This Python file uses the following encoding: utf-8


import serial

import rospy

BAUDRATE = 115200

DLE = 0x90
STX = 0x02

WAITING_DLE = 0
WAITING_STX = 1
WAITING_OPCODE = 2
WAITING_LEN = 3
WAITING_DATA = 4
WAITING_CRC1 = 5
WAITING_CRC2 = 6

MSV2_PROGRESS = 1
MSV2_SUCCESS = 0
MSV2_ERROR = 2
MSV2_WRONG_CRC = 3


def crc16(message):
    crc = 0
    i = 0
    length = len(message)/2
    while(length):
        shifter = 0x8000
        c = message[i] | message[i+1]<<8
        while 1:
            carry = crc & 0x8000
            crc <<= 1
            crc &= 0xffff
            if (c & shifter):
                crc += 1
            if (carry):
                crc ^= 0x1021
            shifter >>= 1
            if(not shifter):
                break
        length -= 1
        i += 2
    return crc


class msv2:
    def __init__(self):
        self.ser = serial.Serial()
        self.escape = 0
        self.connected = 0
        self.state = WAITING_DLE
        self.crc_data = []
        self.data = []


    def connect(self, port):
        self.port = port
        self.ser.baudrate = BAUDRATE
        self.ser.port = port
        self.ser.timeout = 0.1
        try:
            self.ser.open()
            self.connected = 1
            print("connected")
            return 1
        except:
            return 0
    def reconnect(self):
        try:
            self.ser.port = self.port
            self.ser.baudrate = BAUDRATE
            self.ser.timeout = 0.1
            self.ser.open()
            self.connected = 1
            print("connected")
            return 1
        except:
            return 0

    def disconnect(self):
        try:
            self.ser.close()
            self.connected = 0
            print("disconnected")
            return 1
        except:
            return 0

    def is_connected(self):
        return self.connected

    def encode(self, opcode, data):
        bin_data = []
        crc_data = []
        bin_data.append(DLE)
        bin_data.append(STX)
        bin_data.append(opcode)
        bin_data.append(int(len(data)/2))
        crc_data.append(opcode)
        crc_data.append(int(len(data)/2))
        i = 0
        for byte in data:
            bin_data.append(byte)
            crc_data.append(byte)
            if byte == DLE:
                bin_data.append(byte)

        crc_data.append(0)
        crc_data.append(0)
        crc = crc16(crc_data)
        
        bin_data.append(crc & 0x00ff)
        bin_data.append((crc >> 8) & 0x00ff)
        return bin_data


    def decode(self, d):
        #print("decode state:", self.state)
        d = ord(d)
        if (self.escape == 1 and d == STX):
            self.state = WAITING_OPCODE
            self.escape = 0
            return MSV2_PROGRESS

        if (self.state == WAITING_DLE and d == DLE):
            self.crc_data = []
            self.data = []
            self.state = WAITING_STX
            return MSV2_PROGRESS

        if (d == DLE and self.escape == 0):
            self.escape = 1
            return MSV2_PROGRESS

        if (d == DLE and self.escape == 1):
            self.escape = 0

        if (self.state == WAITING_STX and d == STX):
            self.state = WAITING_OPCODE
            return MSV2_PROGRESS

        if (self.state == WAITING_OPCODE):
            self.opcode = d
            self.state = WAITING_LEN
            self.crc_data.append(d)
            return MSV2_PROGRESS

        if (self.state == WAITING_LEN):
            self.data_len = d
            self.length = 2*d
            self.crc_data.append(d)
            self.counter = 0
            self.state = WAITING_DATA
            return MSV2_PROGRESS

        if (self.state == WAITING_DATA):
            self.data.append(d)
            self.crc_data.append(d)
            self.counter += 1
            if (self.counter==self.length):
                self.state = WAITING_CRC1
            return MSV2_PROGRESS

        if (self.state == WAITING_CRC1):
            self.crc = d
            self.state = WAITING_CRC2
            return MSV2_PROGRESS

        if (self.state == WAITING_CRC2):
            self.crc += d<<8
            self.state = WAITING_DLE
            self.crc_data.append(0)
            self.crc_data.append(0)
            if(self.crc == crc16(self.crc_data)):
                return MSV2_SUCCESS
            else:
                return MSV2_WRONG_CRC

        self.state=WAITING_DLE
        return MSV2_PROGRESS

    def send(self, opcode, data):
        if self.connected:
            msg = self.encode(opcode, data)
            error = 0
            try:
                self.ser.write(msg)
            except:
                print("WRITE ERROR")
                self.reconnect()
                return -1
            #print('[{}]'.format(', '.join(hex(x) for x in msg)))
            try:
                while 1:
                    byte = self.ser.read(1)
                    #print("dta: {}".format(hex(ord(byte))))
                    if not byte:
                        print("no resp error")
                        return 0
                    res = self.decode(byte)
                    #print("res:", res)
                    if not res == MSV2_PROGRESS:
                        break
                #print('[{}]'.format(', '.join(hex(x) for x in self.data)))
                if self.data == [0xce, 0xec] or self.data == [0xbe, 0xeb]:
                    print("CRC_ERROR")
                    return 0
                else:
                    print("nominal_resp")
                    return self.data
            except:
                print("READ ERROR")
                self.reconnect()
                return -1
        else:
            print("CONN_ERROR")
            return -1

    def slave(self, callback):
        self.ser.timeout = 10 #no timeout for slave mode
        while rospy.is_shutdown():
            d = self.ser.read(1)
            if not d:
                print("no byte")
                continue
            res = self.decode(d)
            if res == MSV2_SUCCESS:
                callback(self.opcode, self.data)
            elif res == MSV2_WRONG_CRC:
                print("crc error")
    def send_from_slave(self, opcode, data):
        if self.connected:
            msg = self.encode(opcode, data)
            error = 0
            try:
                self.ser.write(msg)
            except:
                print("WRITE ERROR")
                self.reconnect()
                return -1
            
            







