# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 20:09:50 2022

@author: riana
"""

def NavFunc(Period,Family,Nav,NumSC):
    #Inputs: Period: Orbital Period, TBD
    #Orbital Family, TBD
    #Nav: Navigation System, GPS, Optical Communications, Ground Tracking
    #NumSC: Number of Spacecraft, TBD
    
    #Output: Cn: Final cost function value
    
    if Family = 'L2':
        if Period = 14:
            Stability = 23
        if Period = 7: #goes through the moon tho?
            Stability = 1/1.6867100474667822   
        if Period = 29:
            Stability = null
        
    if Family = 'L4':
        if Period = 14:
            Stability = null
        if Period = 7: #goes through the moon tho?
            Stability = null 
        if Period = 29:
            Stability = 1/1
        
    
    if Family = 'l1':
        if Period = 14:
            Stability = 1/524
            Coverage =  #Because never blocked by the moon
        if Period = 7: #goes through the moon tho?
            Stability = null   
        if Period = 29:
            Stability = 1/53
    
    
    
    
    #get values that are dependent on trades
    if Nav = 'GPS':
        if Period = 7 and Family = 'L2':
            DOP = .99*NumSC/8
        elif Period = 12 and Family = 'L2':
            DOP = 1*NumSC/8
        elif Period = 12 and Family = 'L1':
            DOP = 0.1561*NumSC/8
        elif Period = 29 and Family = 'L1':
            DOP = 0.5974*NumSC/8
        elif Period = 29 and Family = 'L4':
            DOP = 0.2505*NumSC/8
            
        Coverage = 1# For all orbit
        TRL = 0.88
    
    if Nav = 'Optic':
        if Period = 7 and Family = 'L2':
            DOP = .99*NumSC/8
        elif Period = 12 and Family = 'L2':
            DOP = 1*NumSC/8
        elif Period = 12 and Family = 'L1':
            DOP = 0.1561*NumSC/8
        elif Period = 29 and Family = 'L1':
            DOP = 0.5974*NumSC/8
        elif Period = 29 and Family = 'L4':
            DOP = 0.2505*NumSC/8
            
        Coverage = 1 #regardless of period
        TRL = 0.88
        
    if Nav = 'Ground':
        DOF = 1 #Does not change as long as coverage is greater than zero
        Coverage = (83.2 - NumSC/2)/100 #for ground stations
        TRL = 1  
    
    
    Cn = 0.2*TRL + 0.3*Coverage + .25*Stability + .25*DOP
    
    return Cn
