import numpy as np


class Probabilistic:
    def __init__(self):
        self.init_bel = np.array(([0.25, 0.25, 0.25, 0.25])).reshape((4, 1))
        self.bel_0 = self.init_bel
        self.bel_bar1 = np.zeros((4, 1))
        self.bel_1 = np.zeros((4, 1))

    def reset_bel(self):
        self.bel_0 = self.init_bel

    def cal_bel_bar(self, p1, p2, p3, p4):
        return self.bel_0[0, :] * p1 + self.bel_0[1, :] * p2 + self.bel_0[2, :] * p3 + self.bel_0[3, :] * p4

    def predict(self, p_z):
        self.bel_bar1[0, :] = self.cal_bel_bar(0.65, 0, 0, 0.1)
        self.bel_bar1[1, :] = self.cal_bel_bar(0, 0.7, 0, 0.2)
        self.bel_bar1[2, :] = self.cal_bel_bar(0, 0, 0.7, 0.1)
        self.bel_bar1[3, :] = self.cal_bel_bar(0.35, 0.3, 0.3, 0.6)
        self.bel_1[0, :] = p_z[:, 0] * self.bel_bar1[0, :]
        self.bel_1[1, :] = p_z[:, 1] * self.bel_bar1[1, :]
        self.bel_1[2, :] = p_z[:, 2] * self.bel_bar1[2, :]
        self.bel_1[3, :] = p_z[:, 3] * self.bel_bar1[3, :]
        eta = 1/(self.bel_1[0, :] + self.bel_1[1, :] + self.bel_1[2, :] + self.bel_1[3, :])
        self.bel_1 = self.bel_1 * eta
        self.bel_0 = self.bel_1
        return self.bel_1
