# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        # initialization of x and P based on unassigned measurement transformed from sensor to vehicle coordinates
       
        # create sensor position vector (px, py, pz, 1) 
        pos_sensor = np.zeros((4, 1))
        pos_sensor[0:3] = meas.z[0:3] # z is the sensor measurement vector
        
        # convert sensor position to vehicle coordinates
        # sense to vehicle matrix is a variable within the sensor class
        pos_veh = np.matmul(meas.sensor.sens_to_veh, pos_sensor)
        
        # initialize track
        self.x = np.zeros((6,1))
        self.x[0:3] = pos_veh[0:3]
        
        # initialization Covariance P = (Ppos|0, 0|Pvel)
        self.P = np.zeros((6, 6))
        
        # Ppos is 3x3 matrix of position uncertainity = Mrot * R * Mrot transpose, where R is measurement covariance for the sensor
        P_pos = np.matmul(np.matmul(M_rot, meas.R), M_rot.transpose())
        self.P[0:3, 0:3] = P_pos
        
        # Pvel is diagonal matrix with large diagonal values
        self.P[3, 3] = params.sigma_p44
        self.P[4, 4] = params.sigma_p55
        self.P[5, 5] = params.sigma_p66
        
        ''' Uncomment when running single track EKF'''
        # self.x = np.matrix([[49.53980697],
        #                 [ 3.41006279],
        #                 [ 0.91790581],
        #                 [ 0.        ],
        #                 [ 0.        ],
        #                 [ 0.        ]])
        # self.P = np.matrix([[9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 6.4e-03, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+01]])
        
        # initialize track state and track score
        self.state = 'initialized'
        self.score = 1./params.window # score updated for 6 windows
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        
        '''implement track management:
        - decrease the track score for unassigned tracks
        - delete tracks if the score is too low or P is too big'''
        
        # decrease score for unassigned tracks
        for i in unassigned_tracks:
            track = self.track_list[i]
            # check visibility 
            if meas_list: # if not empty
                if meas_list[0].sensor.in_fov(track.x):
                    # your code goes here
                    track.score -= 1./params.window

        # delete old tracks
        for track in self.track_list:
            if track.state == 'confirmed' and track.score <= params.delete_threshold:
                    self.delete_track(track)
            elif (track.state =='initialized' or track.state == 'tentative') and \
                (track.P[0,0]>= params.max_P or track.P[1,1] >= params.max_P ) or (track.score <= 0.01):
                    self.delete_track(track)
            
            
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self, track):      
        
        ''' Implement track management for updated tracks:
        - increase track score
        - set track state to 'tentative' or 'confirmed'''
        
        if track.score < 1:
            track.score +=  1./params.window
        
        if track.score >= params.confirmed_threshold:
            track.state = 'confirmed'
        else:
            track.state = 'tentative'
        ############
        # END student code
        ############ 