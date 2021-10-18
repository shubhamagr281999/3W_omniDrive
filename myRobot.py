from math import sin, cos, exp, sqrt, pi
from numpy import array, dot
from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period, wheel_radius=None, L=None):
        Robot.__init__(self, sampling_period, wheel_radius, L)

    # --------------------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 2                                                         #
    # --------------------------------------------------------------------------------------#
    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        r = self._wheel_radius
        psi1=dot(array([sin(theta)/r,-1*cos(theta)/r,-1*L/r]),array(p_dot))
        psi2=dot(array([cos(theta+pi/6)/r,sin(theta+pi/6)/r,-1*L/r]),array(p_dot))
        psi3=dot(array([-1*cos(pi/6-theta)/r,sin(pi/6-theta)/r,-1*L/r]),array(p_dot))

        wheel_angular_velocities=array([[psi1],[psi2],[psi3]])

        return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_forward(self, vy, theta):
        p_dot = array([0, vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

        
    def move_backward(self, vy, theta):        
        p_dot = array([0, -vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
        
    def move_right(self, vx, theta):        
        p_dot = array([vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
        
    def rotate_CCW(self, w, theta):                
        p_dot = array([0.0, 0.0, w]).T
        wd = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(wd)
                
# clockwise is negative as theta is positive ccw
                 
    def rotate_CW(self, w, theta):                
        p_dot = array([0.0, 0.0, -w]).T
        wd = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(wd)
