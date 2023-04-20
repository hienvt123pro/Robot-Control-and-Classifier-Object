"""
Custom data communication protocol setting with structure:
Byte      ||       function       ||       value
0                  (start)                 0
1                  (dev)                   1 or 2
2                  (cmd)                   ...
3                  (cmd)                   ...
4                  (data)                  pick-x
5                  (data)                  pick-y
6                  (data)                  pick-z
7                  (data)                  drop-x
8                  (data)                  drop-y
9                  (data)                  drop-z
10                 (stop)                  1

Example:
    data: 0,1,3,0,16,4,0,0,0,0,1
     + device: 1 - arduino
     + cmd: (3,0) - simple point robot moving command
     + pick-point: (16,4,0) - x,y,z of pick point
     + drop-point: (0,0,0)
"""


class RobotCode:
    def __init__(self):
        self.fullcode = None
        self.CALIBJ1 = "2,1,"
        self.CALIBJ2 = "2,2,"
        self.CALIBJ3 = "2,3,"
        self.CALIBHOME = "2,4,"
        self.MOVE = "3,0,"
        self.SCON = "0,0,"
        self.CHECK_STATE = "1,0,"
        self.PROCESS_MOVE = "4,0,"
        self.SET_INTER_POINT = "5,0,"
        self.EFFECTOR = '6,0,'
        self.RUNCONVEY = '7,0,'
        self.STOPCONVEY = "7,1,"
        self.READSPEED = "8,0,"
        self.ERROR_SIZE = "9,0,"

    def concatenate(self, dev: str, cmd: str, data: list):
        if not data:
            data = ["0", "0", "0", "0", "0", "0"]

        if not dev:
            dev = "1"

        self.fullcode = "0," + str(dev) + "," + cmd + str(data[0]) + "," + str(data[1]) + "," + str(data[2]) + ","\
                        + str(data[3]) + "," + str(data[4]) + "," + str(data[5]) + "," + "1,"
        return self.fullcode


robocod = RobotCode()
