import serial
from cobs import cobsr
import argparse
from enum import Enum
from dataclasses import dataclass
import time, threading
import random
import math
import time

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
        self.ci = random.randrange(254)
        self.receiveCounter = 0

    def compileMessage(self, cmd, code, data=bytearray(), ACK=0, CI=0):
        if CI==0:
            self.ci = (self.ci + 1)%254
        else:
            self.ci = CI-1

        sendCi = self.ci+1   # CI only goes from 1 to 255, not 0 to 255.

        newMsg = bytearray()

        newMsg += b'\x00'                            # Start with 0x00
        newMsg.append(((ACK & 1) << 7) | ord(cmd))   # ACK or noACK encoded with cmd
        newMsg.append(sendCi)                            # continuity indicator
        newMsg.append(len(data))
        newMsg.append(code)
        newMsg += data

        # Calculate Checksum
        msgSum = 0
        for x in newMsg:
            msgSum += x

        check = (256 - (msgSum % 256)) % 256
        newMsg.append(check)

        # encode part of MSG. Starting at code
        newMsg = newMsg[:4] + cobsr.encode(newMsg[4:])

        # replace data length with length of encoded part
        newMsg[3] = len(newMsg[4:])

    #    print(newMsg)
        return newMsg




    def parse(self, newCharacter, verbose):
        self.lastPacket.raw.append(ord(newCharacter))

        if newCharacter == b'\x00':
            self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
            self.lastPacket.raw.append(ord(newCharacter))

            self.receiveCounter += 1
            if verbose: print('\n'+str(self.receiveCounter),end='< ')
            self.state = ParserStates.waitForCMD

        elif self.state == ParserStates.waitForCMD:
            self.lastPacket.ACK = (ord(newCharacter) & 0b10000000) >> 7
            self.lastPacket.CMD = chr( ord(newCharacter) & 0b01111111 )

            if verbose: print('ACK:'+str(self.lastPacket.ACK),end=' ')
            if verbose: print('CMD:' + str(self.lastPacket.CMD),end=' ')
            self.state = ParserStates.waitForCI

        elif self.state == ParserStates.waitForCI:
            self.lastPacket.CI = ord(newCharacter)

            if verbose: print('CI:' + "{:02x}".format(self.lastPacket.CI),end=' ')
            self.state = ParserStates.waitForLEN

        elif self.state == ParserStates.waitForLEN:
            self.lastPacket.LEN = ord(newCharacter)

            self.expectedData = self.lastPacket.LEN
            self.lastPacket.encodedData = bytearray()

            if verbose: print('LEN:' + "{:02x}".format(self.lastPacket.LEN),end=' ')
            self.state = ParserStates.collectEncodedData

        elif self.state == ParserStates.collectEncodedData and self.expectedData != 0:
            self.expectedData -= 1
            if self.expectedData == 0:
                self.lastPacket.rawDecoded = self.lastPacket.raw[:4] + cobsr.decode(self.lastPacket.raw[4:])
                self.lastPacket.code = "{:02x}".format(self.lastPacket.rawDecoded[4])
                dataAsInteger = int.from_bytes(self.lastPacket.rawDecoded[5:-1], byteorder='little')

                if self.lastPacket.code=='26':
                    if verbose: print('Text:"'+self.lastPacket.rawDecoded[5:-1].decode("utf-8")+'"',end=' ')
                elif self.lastPacket.code=='fe':
                    if verbose: print('ProtocolVersion:'+str(dataAsInteger),end=' ')
                elif self.lastPacket.code=='27':
                    if verbose: print('Ping:'+str(int(round(time.time() * 1000)) - dataAsInteger)+'ms',end=' ')
                else:
                    if verbose: print('Code:'+self.lastPacket.code,end=' ')
                    if self.lastPacket.CMD == 'w':
                        if verbose: print('Write:'+str(dataAsInteger),end=' ')
                    else:
                        if verbose: print(self.lastPacket.rawDecoded[5:-1],end=' ')

                self.lastPacket.CS = "{:02x}".format(self.lastPacket.rawDecoded[-1])
                if verbose: print('CS:'+self.lastPacket.CS,end=' ')

                self.state = ParserStates.idle

        else:
            if verbose: print('Unknown Character:', newCharacter)


        return self.lastPacket


class PeriodicAction:
    """Perform periodic action"""

    def __init__(self, code):
        self.deg = 0
        self.code = code
        self.sentCounter = 0

    def periodicFunctions(self):
    #    ser.write( parseProtocol.compileMessage('R',0xFE) )

        data  = bytearray()
        if self.code == 0x27:
            data += int(round(time.time() * 1000)).to_bytes(length=8,byteorder='little', signed=True)
        else:
            data += int(math.sin(math.radians(self.deg))*200).to_bytes(length=4,byteorder='little', signed=True)
            data += int(math.sin(math.radians(self.deg))*-200).to_bytes(length=4,byteorder='little', signed=True)
            self.deg += 2
            if self.deg > 360:
                self.speed = 0

        ser.write( parseProtocol.compileMessage('W', self.code, data) )
        self.sentCounter += 1
        print('\n'+str(self.sentCounter),end='> ')
        threading.Timer(0.01, self.periodicFunctions).start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Enter Serial Port. Example: COM5 or /dev/ttyUSB0", type=str)
    parser.add_argument("-b", "--baudrate", help="Baudrate, default 38400", type=int)
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("--send100ms", help="send a command every 100ms with sinus data. Enter code as hex like: 0x0D")
    args = parser.parse_args()

    if args.baudrate:
        baudrate = args.baudrate
    else:
        baudrate = 115200

    verbose = args.verbose

    ser = serial.Serial(args.port, baudrate = baudrate)  # open serial port

    parseProtocol = ParseProtocol()

    if args.send100ms:
        periodic = PeriodicAction(int(args.send100ms, base=16))
        periodic.periodicFunctions()

    while(1):
        receivedPacket = parseProtocol.parse(ser.read(), verbose)
      #  if receivedPacket.rawDecoded != bytearray():
       #     print(receivedPacket)
    #        print('ACK', receivedPacket.ACK, 'CMD', receivedPacket.CMD, 'CI', receivedPacket.CI, 'LEN', receivedPacket.LEN, 'code', receivedPacket.code, receivedPacket.rawDecoded)


# "args": ["-v", "--send100ms", "0x0D", "/dev/ttyUSB0"]