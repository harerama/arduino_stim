"""
Code licensed under the GNU Affero General Public License v3 or later (AGPLv3+)
"""

"""
Handles serial communication with ArduinoStim device
"""

from enum import Enum, unique
import struct
import logging
import threading
import time
import serial
import PySide
import serial.tools.list_ports


@unique
class MessageType(Enum):
    actual_current = 1
    target_current = 2
    safety_limit = 3
    absolute_limit = 4
    scaling = 5
    max_power_for_channel = 6
    current_override = 7

    playback_speed = 10
    playback_power = 11

    ramping_speed = 20
    ramping_limit = 21

    tap_value = 30
    voltage = 31

    start = 100
    stop = 101
    start_playback = 102
    stop_playback = 103
    start_diagnostic = 104
    stop_diagnostic = 105

    bad_packet = 255


"""
Message format:
  - 1 byte header (currently 0)
  - 1 byte length (data + checksum)
  - n bytes of data
  - 1 byte of checksum (XOR of all bytes excluding checksum)
"""


class SerialMessage:
    def __init__(self, data):
        self.data = data

    def len(self):
        return len(self.data) + 1

    def checksum(self):
        res = 0
        res ^= 0
        res ^= self.len()
        for d in self.data:
            res ^= d
        return res

    def send(self, port):
        msg = [0, self.len()]
        msg.extend(self.data)
        msg.append(self.checksum())
        port.write(bytearray(msg))


class ArduinoSerial:
    def set_float_parameter_limit(self, ptype, value):
        data = [ptype]
        ArduinoSerial.add_float(data, value)
        msg = SerialMessage(data)
        msg.send(self.port)

    def stop(self):
        logging.info("Stopping stimulation")

    def handle_data(self, data):
        index = 0
        values = {}
        while len(data) - index > 4:
            if data[index] == MessageType.bad_packet.value:
                logging.warning("Bad packet received by device", list(data))
            else:
                value = struct.unpack_from('<f', data, index + 1)
                values[data[index]] = value[0]
                index += 5
        if values:
            self.signal.emit(values)

    def read_from_port(self):
        while True:
            if self.port is not None and self.port.inWaiting() > 0:
                chk = 0
                heading = self.port.read(1)
                chk ^= heading[0]
                length = self.port.read(1)[0]
                chk ^= length
                if length > 0:
                    data = self.port.read(length)
                    if len(data) != length:
                        logging.warning("Invalid length")
                    for b in data:
                        chk ^= b
                    if chk != 0:
                        logging.warning("Bad packet received by computer ", list(data))
                        self.port.flushInput()
                    else:
                        self.handle_data(data)
                else:
                    logging.warning("Empty packet")
                    self.port.flushInput()
            else:
                time.sleep(0.01)

    def __init__(self, signal):
        self.port = None
        self.signal = signal
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.setDaemon(True)  # Makes sure the thread is killed when window is closed
        self.thread.start()

    def connect_to_port(self, port):
        logging.info("Connecting to " + port)
        self.port = serial.serial_for_url(port, baudrate=9600)

        if port == "loop://":
            PySide.QtCore.QTimer.singleShot(5000, self.sendDummyUpdate)


    @staticmethod
    def add_float(data, value):
        s = struct.pack('<f', value / 1000)
        l = struct.unpack('<BBBB', s)
        data.extend(l)

    def sendDummyUpdate(self):
        dummyData = []
        dummyData.append(MessageType.actual_current.value)
        ArduinoSerial.add_float(dummyData, 0.001)
        dummyData.append(MessageType.target_current.value)
        ArduinoSerial.add_float(dummyData, 0.002)
        SerialMessage(dummyData).send(self.port)