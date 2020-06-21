import serial
from cobs import cobsr
from enum import Enum
from struct import *
from dataclasses import dataclass

class ParserStates(Enum):
    idle    = 0
    waitForCMD  = 1
    waitForCI   = 2
    waitForLEN  = 3
    collectEncodedData  = 4

@dataclass
class BipropellantPacket:
    raw:        bytearray
    rawDecoded: bytearray
    ACK:        int
    CMD:        chr
    CI:         int
    LEN:        int
    code:       int
    CS:         int

class ParseProtocol:
    """Parse Bipropellant Protocol"""

    def __init__(self):
        self.state = ParserStates.idle
        self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
        self.expectedData = 0

    def parse(self, newCharacter):
        self.lastPacket.raw.append(ord(newCharacter))

        if newCharacter == b'\x00':
            self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
            self.lastPacket.raw.append(ord(newCharacter))

            print(end = '\n> ')
            self.state = ParserStates.waitForCMD

        elif self.state == ParserStates.waitForCMD:
            self.lastPacket.ACK = (ord(newCharacter) & 0b10000000) >> 7
            self.lastPacket.CMD = chr( ord(newCharacter) & 0b01111111 )

            print('ACK:'+str(self.lastPacket.ACK),end=' ')
            print('CMD:' + str(self.lastPacket.CMD),end=' ')
            self.state = ParserStates.waitForCI

        elif self.state == ParserStates.waitForCI:
            self.lastPacket.CI = ord(newCharacter)

            print('CI:' + "{:02x}".format(self.lastPacket.CI),end=' ')
            self.state = ParserStates.waitForLEN

        elif self.state == ParserStates.waitForLEN:
            self.lastPacket.LEN = ord(newCharacter)

            self.expectedData = self.lastPacket.LEN
            self.lastPacket.encodedData = bytearray()

            print('LEN:' + "{:02x}".format(self.lastPacket.LEN),end=' ')
            self.state = ParserStates.collectEncodedData

        elif self.state == ParserStates.collectEncodedData and self.expectedData != 0:
            self.expectedData -= 1
            if self.expectedData == 0:
                self.lastPacket.rawDecoded = self.lastPacket.raw[:4] + cobsr.decode(self.lastPacket.raw[4:])
                self.lastPacket.code = "{:02x}".format(self.lastPacket.rawDecoded[4])
                if self.lastPacket.code=='26':
                    print('Text:"'+self.lastPacket.rawDecoded[5:-1].decode("utf-8")+'"',end=' ')
                elif self.lastPacket.code=='fe':
                    version = int.from_bytes(self.lastPacket.rawDecoded[5:-1], byteorder='little')
                    print('ProtocolVersion:'+str(version),end=' ')
                else:
                    print('Code:'+self.lastPacket.code,end=' ')
                    print(self.lastPacket.rawDecoded[5:-1],end=' ')

                self.lastPacket.CS = "{:02x}".format(self.lastPacket.rawDecoded[-1])
                print('CS:'+self.lastPacket.CS,end=' ')

                self.state = ParserStates.idle

        else:
            print(hex, end = ' ')

        return self.lastPacket


parseProtocol = ParseProtocol()
ser = serial.Serial('/dev/ttyUSB0', baudrate = 115200)  # open serial port

while(1):
    receivedPacket = parseProtocol.parse(ser.read())
    if receivedPacket.rawDecoded != bytearray():
        print(receivedPacket)