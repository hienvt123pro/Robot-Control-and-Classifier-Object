"""
Apply RandomForestRegressor algorithm to predict the robot's pick-up point in the future based on the following factors:
- Velocity of conveyor (as well as velocity of object): V_conveyor
- Position of the center point at the initial time: Y_init
- Operating system delay time = time down of robot + time detecting object + time running model rf: T_delay
Linear motion model: The future point = Y_init + V_conveyor * T_delay
"""

import pickle
import numpy as np
from sklearn.ensemble import RandomForestRegressor


class RandomForestPredictPoint:
    def __init__(self):
        self.CONVEYOR_VELOCITY = 3  # (cm/sec)
        self.T_ROBOT = 1.3  # time robot down (sec)
        self.T_DELAY = 0.28 + 0.0155  # time_detect + time_run_rf (sec)
        self.AVERAGE_SYS_DELAY_TIME = 2.2955  # (sec)
        self.LOW_WORKING_Y_AREA = 0  # (cm)
        self.HIGH_WORKING_Y_AREA = 2  # (cm)
        self.SAMPLES_DIM = 200

        with open('models/reg_point.pkl', 'rb') as f:
            self.rfmodel = pickle.load(f)

    def predict_new_point(self, y_init_point, sys_delay_time):
        y_future_point = self.rfmodel.predict(np.array([[y_init_point, sys_delay_time]]))
        return round(y_future_point[0], 2)

    def create_new_model(self):
        conveyor_velocity_norm = np.random.normal(loc=self.CONVEYOR_VELOCITY, scale=0.15, size=self.SAMPLES_DIM)
        time_robot_norm = np.random.normal(loc=self.T_ROBOT, scale=0.2, size=self.SAMPLES_DIM)
        time_delay_norm = np.random.normal(loc=self.T_DELAY, scale=0.5, size=self.SAMPLES_DIM)

        total_delay_time = time_robot_norm + time_delay_norm
        y_current = np.random.uniform(low=self.LOW_WORKING_Y_AREA, high=self.HIGH_WORKING_Y_AREA, size=(self.SAMPLES_DIM,))
        y_future = y_current + conveyor_velocity_norm * total_delay_time

        y_current = np.array([y_current]).T
        delay_time = np.array([total_delay_time]).T
        y_future = np.array(y_future)

        X_train = np.concatenate((y_current, delay_time), axis=1)
        y_train = y_future

        rfmodel = RandomForestRegressor(n_estimators=100, random_state=45)
        rfmodel.fit(X_train, y_train)

        with open('models/reg_point.pkl', 'wb') as f:
            pickle.dump(rfmodel, f)

        with open('models/reg_point.pkl', 'rb') as f:
            self.rfmodel = pickle.load(f)


rfPoint = RandomForestPredictPoint()
