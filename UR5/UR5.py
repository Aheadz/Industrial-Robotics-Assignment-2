##  @file
#   @brief UR5 Robot defined by standard DH parameters with 3D model
#   @author Ho Minh Quang Ngo
#   @date Jul 20, 2023

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class UR5(DHRobot3D):
    """ 
    Example usage:
    >>> from ir-support import UR5
    >>> import swift

    >>> r = UR5()
    >>> q = [0,-pi/2,pi/4,0,0,0]
    >>> r.q = q
    >>> q_goal = [r.q[i]-pi/4 for i in range(r.n)]
    >>> env = swift.Swift()
    >>> env.launch(realtime= True)
    >>> r.add_to_env(env)
    >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
    >>> for q in qtraj:
    >>>    r.q = q
    >>>    env.step(0.02)

    """     
    def __init__(self):         
        # DH links
        links = self._create_DH()     

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_ur5', 
                            link1 = 'shoulder_ur5', 
                            link2 = 'upperarm_ur5', 
                            link3 = 'forearm_ur5', 
                            link4 = 'wrist1_ur5', 
                            link5 = 'wrist2_ur5', 
                            link6 = 'wrist3_ur5')

        # A joint config and the 3D object transforms to match that config
        qtest = [0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(0,0,0),                                                     
                            spb.transl(0,0,0.086139) @ spb.trotz(pi), 
                            spb.transl(0,-0.13638,0.086067) @ spb.trotz(pi),
                            spb.transl(0,-0.016547,0.51077) @ spb.trotz(pi), 
                            spb.transl(0,-0.016682,0.90308) @ spb.rpy2tr(0,-pi/2,pi, order ='xyz'),
                            spb.transl(0.00123,-0.10987,0.903) @ spb.rpy2tr(0,-pi/2,pi, order= 'xyz'),
                            spb.transl(-0.093542,-0.11018,0.90314) @ spb.trotx(pi)]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'UR5', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links
                    
    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)      
        self.q = self._qtest  
        # self.base = SE3(0.5,0.5,0)
        self.add_to_env(env)
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        # env.hold()
        time.sleep(3)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r = UR5()
    r.test()

    