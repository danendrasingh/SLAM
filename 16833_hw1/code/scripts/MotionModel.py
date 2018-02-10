import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """



    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """
        alpha1=alpha2=alpha3=alpha4=0.0

        del_trans=math.sqrt( math.pow( u_t1[1] - u_t0[1], 2) + math.pow( u_t1[0] - u_t0[0], 2) )
        del_rot1=math.atan2(( u_t1[1] - u_t0[1] ),( u_t1[0] - u_t0[0] )) - u_t0[2]
        del_rot2=u_t1[2] - u_t0[2] - del_rot1

        sq_del_rot1=math.pow(del_rot1,2)
        sq_del_rot2=math.pow(del_rot2,2)
        sq_del_trans=math.pow(del_trans,2)

        delhat_rot1=del_rot1-np.random.normal(0,np.multiply(alpha1,sq_del_rot1)+np.multiply(alpha2,sq_del_trans))
        delhat_trans=del_trans-np.random.normal(0,np.multiply(alpha3,sq_del_trans)+np.multiply(alpha4,sq_del_rot1)+ np.multiply(alpha4,sq_del_rot2))        
        delhat_rot2= del_rot2-np.random.normal(0,np.multiply(alpha1,sq_del_rot2)+np.multiply(alpha2,sq_del_trans))

        
        x_t1=np.empty(3)
        x_t1[0]= x_t0[0] + np.multiply(delhat_trans,math.cos(x_t0[2]+delhat_rot1))      
        x_t1[1]= x_t0[1] + np.multiply(delhat_trans,math.sin(x_t0[2]+delhat_rot1))
        x_t1[2]= x_t0[2] + delhat_rot1+ delhat_rot2


        return x_t1

if __name__=="__main__":
    pass