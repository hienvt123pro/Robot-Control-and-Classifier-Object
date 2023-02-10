class RobotCode:
    def __init__(self):
        self.fullcode = None
        self.CALIBJ1 = "2,1,"
        self.CALIBJ2 = "2,2,"
        self.CALIBJ3 = "2,3,"
        self.CALIBHOME = "2,4,"
        self.MOVE = "3,0,"
        self.SCON = "0,0,"

    def concatenate(self, dev, cmd, data):
        if not data:
            data = ["0", "0", "0", "0", "0", "0"]

        if not dev:
            dev = "1"

        self.fullcode = "0," + str(dev) + "," + cmd + str(data[0]) + "," + str(data[1]) + "," + str(data[2]) + ","\
                        + str(data[3]) + "," + str(data[4]) + "," + str(data[5]) + "," + "1,"
        return self.fullcode


robocod = RobotCode()
