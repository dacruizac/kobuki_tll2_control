#!/usr/bin/python

import numpy as np

class controller(object):
    def __init__(self,k_alpha,k_phi,lim_v,lim_w):
        self.k_alpha=k_alpha
        self.k_phi=k_phi
        self.lim_v=lim_v
        self.lim_w=lim_w
    
    def linear_control(self,phi,alpha):
        c_signal=self.k_phi*phi*np.cos(alpha)
        c_signal=min(c_signal,self.lim_v)
        c_signal=max(c_signal,-self.lim_v)
        return c_signal

    def angular_control(self,phi,alpha):
        c_signal=self.k_phi*np.cos(alpha)*np.sin(alpha) + self.k_alpha*alpha
        c_signal=min(c_signal,self.lim_w)
        c_signal=max(c_signal,-self.lim_w)
        return c_signal