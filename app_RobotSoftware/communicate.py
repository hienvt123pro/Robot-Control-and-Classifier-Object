import threading
import serial
import time
import serial.tools.list_ports


class SerialCommunication:
    """
    Provide functions to connect com port, communicate with MCU
    """
    def __init__(self):
        self.port = serial.Serial()
        self.list_ports = serial.tools.list_ports
        self.receive_buff = b'00000'
        self.isKillThread = False

    def list_ports_name(self):
        return self.list_ports.comports()

    def connect(self, port_name: str, baud: str):
        try:
            self.port.port = port_name
            self.port.baudrate = baud
            self.port.open()
            return True
        except:
            return False

    def disconnect(self):
        self.port.close()

    def send_data(self, buffer):
        try:
            self.port.write(buffer.encode("utf-8"))
            time.sleep(0.01)
            self.port.flush()
            return True
        except:
            return False

    def receive_handler(self):
        while True:
            if self.port.isOpen():
                try:
                    if self.port.inWaiting():
                        self.receive_buff = self.port.readline(5)
                except:
                    pass
            if self.isKillThread:
                self.isKillThread = False
                break


# device 1
serialCom1 = SerialCommunication()
receive_thread_1 = threading.Thread(target=serialCom1.receive_handler)

# device 2
serialCom2 = SerialCommunication()
receive_thread_2 = threading.Thread(target=serialCom2.receive_handler)
