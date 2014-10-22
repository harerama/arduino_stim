#!/usr/bin/python

"""
Code licensed under the GNU Affero General Public License v3 or later (AGPLv3+)
"""
import sys
import glob
import serial

from collections import deque

import serial.tools.list_ports
from PySide.QtGui import *
from arduinoSerial import *
from arduinoSerial import MessageType
from arduino_stim.arduinoStim import *


class MainWindow(QDialog, Ui_ArduinoStimControl):
    connected = False
    lastCompleteUpdate = 0
    signal = QtCore.Signal(dict)
    last_current_values = deque([], 500)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.targetDoubleSpinBox.setEnabled(False)
        self.maxCurrentMADoubleSpinBox.setEnabled(False)
        self.scalingDoubleSpinBox.setEnabled(False)
        self.stopButton.setEnabled(False)
        self.serial = ArduinoSerial(self.signal)
        self.signal.connect(self.values_received)

        ports = ["Select port..."] + list(serial_ports()) + ["loop://"]
        for port in ports:
            self.comPort.addItem(port)

        self.comPort.currentIndexChanged.connect(lambda idx: self.serial.connect_to_port(ports[idx]))

        updateuitimer = QtCore.QTimer(self)
        updateuitimer.timeout.connect(self.regular_update)
        updateuitimer.start(1000)

    def regular_update(self):
        if time.time() - self.lastCompleteUpdate > 2:
            self.set_disconnected()
        if len(self.last_current_values) > 0:
            self.currentLineEdit.setText("{0:4g}".format(self.last_current_values[-1]))

    def set_connected(self):
        if not self.connected:
            logging.info("Connected")
            self.targetDoubleSpinBox.setEnabled(True)
            self.maxCurrentMADoubleSpinBox.setEnabled(True)
            self.scalingDoubleSpinBox.setEnabled(True)
            self.stopButton.setEnabled(True)
            self.targetDoubleSpinBox.valueChanged.connect(
                lambda x: self.serial.set_float_parameter_limit(MessageType.target_current.value, x))
            self.maxCurrentMADoubleSpinBox.valueChanged.connect(
                lambda x: self.serial.set_float_parameter_limit(MessageType.safety_limit.value, x / 1000))
            self.scalingDoubleSpinBox.valueChanged.connect(lambda x: self.serial.set_float_parameter_limit(MessageType.scaling.value, x))
            self.stopButton.clicked.connect(self.stop)
            self.connected = True
        self.lastCompleteUpdate = time.time()

    def set_disconnected(self):
        if self.connected:
            self.connected = False
            logging.info("Disconnected")
            self.targetDoubleSpinBox.setEnabled(False)
            self.maxCurrentMADoubleSpinBox.setEnabled(False)
            self.scalingDoubleSpinBox.setEnabled(False)
            self.stopButton.setEnabled(False)
            try:
                self.stopButton.clicked.disconnect()
                self.targetDoubleSpinBox.valueChanged.disconnect()
                self.maxCurrentMADoubleSpinBox.valueChanged.disconnect()
                self.scalingDoubleSpinBox.valueChanged.disconnect()
            except RuntimeError:
                logging.warning("Cannot disconnect signals")

    # Value received from the device
    def value_received(self, msgtype, value):
        if msgtype == MessageType.actual_current.value:
            self.last_current_values.append(value)
        elif msgtype == MessageType.safety_limit.value:
            self.limitLineEdit.setText("{0:4g}".format(value))
        elif msgtype == MessageType.voltage.value:
            self.voltageLineEdit.setText("{0:4g}".format(value))
        elif msgtype == MessageType.target_current.value:
            self.actualTargetLineEdit.setText("{0:4g}".format(value))
        elif msgtype == MessageType.tap_value.value:
            self.tapValueLineEdit.setText("{0:4g}".format(value))
        elif msgtype == MessageType.scaling.value:
            self.actualScalingLineEdit.setText("{0:4g}".format(value))
        elif msgtype == MessageType.max_power_for_channel.value:
            self.actualMaxLineEdit.setText("{0:4g}".format(value))

    # Value received from the device
    def values_received(self, values):
        for param in values:
            self.value_received(param, values[param])
        if len(values) > 1:
            self.set_connected()

    def stop(self):
        self.stopButton.setEnabled(False)
        self.startButton.setEnabled(not self.stopButton.isEnabled())
        self.serial.stop()


def serial_ports():
    """Lists serial ports

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

logging.basicConfig(level=logging.INFO)
if __name__ == '__main__':
    app = QApplication(sys.argv)
    frame = MainWindow()

    frame.show()
    res = app.exec_()
    sys.exit(res)