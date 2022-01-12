# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state
        self.dt = params.dt
        self.q = params.q

    def F(self):
        """Implements and returns system matrix F"""
        # model is assumed to be constant velocity
        dt = self.dt
        
        return np.matrix([[1, 0, 0, dt, 0, 0],
                        [0, 1, 0, 0, dt, 0],
                        [0, 0, 1, 0, 0 , dt],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]])

    def Q(self):
        """implement and return process noise covariance Q"""
        
        # Q(dt) = integral (F*Q*F.transpose()), where q in Q only affects the velocity
        
        q = self.q  # process noise variable
        dt = self.dt
        q1 = ((dt**3)/3) * q 
        q2 = ((dt**2)/2) * q 
        q3 = dt * q 
        return np.matrix([[q1, 0, 0, q2, 0, 0],
                        [0, q1, 0, 0, q2, 0],
                        [0, 0, q1, 0, 0, q2],
                        [q2, 0, 0, q3, 0, 0],
                        [0, q2, 0, 0, q3, 0],
                        [0, 0, q2, 0, 0, q3]])

    def predict(self, track):
        
        """Predict state x and estimation error covariance P to next timestep, save x and P in track """
        
        x_minus = np.matmul(self.F(), track.x)
        P_minus = np.matmul(np.matmul(self.F(), track.P), self.F().transpose()) + self.Q()

        # save predicted state x and covariance P to track 
        track.set_x(x_minus)
        track.set_P(P_minus)

    def update(self, track, meas):
        """ update state x and covariance P with associated measurement, save x and P in track"""
        
        # K = P-*H jacobian transpose * residual covariance inverse
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P*H.transpose()*np.linalg.inv(S)
        
        # updated x = predicted x + K * gamma
        x_plus = track.x + K * self.gamma(track, meas)
        
        # updated P = (I - K*H jacobian) * P-
        I = np.identity(self.dim_state)
        P_plus = (I - K*H)*track.P
        
        track.set_x(x_plus)
        track.set_P(P_plus)
        
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        """calculate and return residual gamma"""
        # gamma = measurement_z - h(x)
        
        y = meas.z - meas.sensor.get_hx(track.x)
        
        return y

    def S(self, track, meas, H):
        """calculate and return covariance of residual S """
        
        # S = H*prediction covariance P * H.transpose()+ measurement covariance R

        return H*track.P*H.transpose() + meas.R