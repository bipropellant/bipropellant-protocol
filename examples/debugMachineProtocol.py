import serial
from cobs import cobsr
import argparse
from enum import Enum
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

    def compileMessage(self, ACK, cmd, CI, code, data):

        newMsg = bytearray()

        newMsg += b'\x00'                            # Start with 0x00
        newMsg.append(((ACK & 1) << 7) | ord(cmd))   # ACK or noACK encoded with cmd
        newMsg.append(CI)                            # continuity indicator
        newMsg.append(len(data))
        newMsg.append(code)
        newMsg += data

        # Calculate Checksum
        CS = 256

        i = 0
        while i < len(newMsg):
            CS -= newMsg[i]
            i += 1

        while CS < 0:
            CS += 256

        newMsg.append(CS)

        # encode part of MSG. Starting at code
        newMsg = newMsg[:4] + cobsr.encode(newMsg[4:])

        # replace data length with length of encoded part
        newMsg[3] = len(newMsg[4:])

        print(newMsg)
        return newMsg




    def parse(self, newCharacter, verbose):
        self.lastPacket.raw.append(ord(newCharacter))

        if newCharacter == b'\x00':
            self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
            self.lastPacket.raw.append(ord(newCharacter))

            if verbose: print(end = '\n> ')
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
                if self.lastPacket.code=='26':
                    if verbose: print('Text:"'+self.lastPacket.rawDecoded[5:-1].decode("utf-8")+'"',end=' ')
                elif self.lastPacket.code=='fe':
                    version = int.from_bytes(self.lastPacket.rawDecoded[5:-1], byteorder='little')
                    if verbose: print('ProtocolVersion:'+str(version),end=' ')
                else:
                    if verbose: print('Code:'+self.lastPacket.code,end=' ')
                    if verbose: print(self.lastPacket.rawDecoded[5:-1],end=' ')

                self.lastPacket.CS = "{:02x}".format(self.lastPacket.rawDecoded[-1])
                if verbose: print('CS:'+self.lastPacket.CS,end=' ')

                self.state = ParserStates.idle

        else:
            if verbose: print('Unknown Character:', newCharacter)


        return self.lastPacket



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Enter Serial Port. Example: COM5 or /dev/ttyUSB0", type=str)
    parser.add_argument("-b", "--baudrate", help="Baudrate, default 38400", type=int)
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()

    if args.baudrate:
        baudrate = args.baudrate
    else:
        baudrate = 115200

    verbose = args.verbose

    ser = serial.Serial(args.port, baudrate = baudrate)  # open serial port

    parseProtocol = ParseProtocol()

    ser.write( parseProtocol.compileMessage(1,'R',2,0xFE,bytearray()) )

    while(1):
        verbose = True
        receivedPacket = parseProtocol.parse(ser.read(), verbose)
        if receivedPacket.rawDecoded != bytearray():
            print(receivedPacket)
    #        print('ACK', receivedPacket.ACK, 'CMD', receivedPacket.CMD, 'CI', receivedPacket.CI, 'LEN', receivedPacket.LEN, 'code', receivedPacket.code, receivedPacket.rawDecoded)