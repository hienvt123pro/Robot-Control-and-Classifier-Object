import threading
import serial
import time
import serial.tools.list_ports


class SerialCommunication:
    def __init__(self):
        self.port = serial.Serial()
        self.list_ports = serial.tools.list_ports
        self.receive_buff = "0,0,0,0,0"

    def list_ports_name(self):
        return self.list_ports.comports()

    def connect(self, port_name, baud):
        try:
            self.port.port = port_name
            self.port.baudrate = baud
            self.port.open()
            return 1
        except:
            return 0

    def disconnect(self):
        self.port.close()

    def send_data(self, buffer):
        try:
            self.port.write(buffer.encode("utf-8"))
            time.sleep(0.01)
            self.port.flush()
            return 1
        except:
            return 0

    def receive_handler(self):
        while True:
            if self.port.isOpen():
                if self.port.inWaiting():
                    self.receive_buff = self.port.readline(5)


serialCom = SerialCommunication()
receive_thread = threading.Thread(target=serialCom.receive_handler)
