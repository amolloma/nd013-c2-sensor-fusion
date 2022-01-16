# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):    
        
        '''Step 3: Association Matrix (N X M):
        - N = number of tracks, M = number of measurements
        - association matrix based on Mahalanobis distance for all tracks and all measurements
        - update list of unassigned measurements and unassigned tracks'''
       
        N = len(track_list)
        M = len(meas_list)
        self.association_matrix = np.full([N, M], np.inf) # (N x M inf matrix)
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas = list(range(M))
        
        for i in range(N): 
            track = track_list[i]
            for j in range(M):
                meas = meas_list[j]
                dist = self.MHD(track, meas, KF)
                if self.gating (dist, meas.sensor):
                    self.association_matrix[i,j] = dist
                
    def get_closest_track_and_meas(self):
        
        '''find closest track and measurement:
        - return NAN if no more associations can be found (i.e. minimum entry in association matrix is infinity)
        - find minimum entry in association matrix, delete associated row and column
        - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        - return this track and measurement'''
    
        # find closest track and measurement for next update
        A = self.association_matrix
        if np.min(A) == np.inf:
            return np.nan, np.nan

        # get indices of minimum entry
        ij_min = np.unravel_index(np.argmin(A, axis=None), A.shape) 
        ind_track = ij_min[0]
        ind_meas = ij_min[1]

        # delete row and column for next update
        A = np.delete(A, ind_track, 0) 
        A = np.delete(A, ind_meas, 1)
        self.association_matrix = A
        
        # update this track with this measurement
        update_track = self.unassigned_tracks[ind_track] 
        update_meas = self.unassigned_meas[ind_meas]
        
        # remove this track and measurement from list
        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)
        
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        
        '''return True if measurement lies inside gate, otherwise False'''
        p = params.gating_threshold
        df = sensor.dim_meas
        
        limit = chi2.ppf(p, df) 
        
        if MHD < limit:
            return True
        else:
            return False
        
    def MHD(self, track, meas, KF):
        
        '''Calculate and return Mahalanobis distance'''
        # Formula MHD = gamma.transpose()* S.inverse * gamma
        
        gamma = KF.gamma(track, meas) # residual
        H = meas.sensor.get_H(track.x) # measurement function
        S = KF.S(track, meas, H) + meas.R # residual covariance
        
        mhd = gamma.transpose()*np.linalg.inv(S)*gamma
        
        return mhd
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)