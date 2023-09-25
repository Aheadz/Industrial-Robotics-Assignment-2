##  @file
#   @brief UR3 Robot defined by standard DH parameters with 3D model
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
class UR3(DHRobot3D):
    """ 
    Example usage:
    >>> from ir-support import UR3
    >>> import swift

    >>> r = UR3()
    >>> q = [0,-pi/2,pi/4,0,0,0]r
    >>> r.q = q
    >>> q_goal = [r.q[i]-pi/4 for i in range(r.n)]
    >>> env = swift.Swift()
    >>> env.launch(realtime= True)
    >>> r.add_to_env(env)
    >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
    >>> for q in qtraj:r
    >>>    r.q = q
    >>>    env.step(0.02)

    """     
    def __init__(self):         
        # DH links
        links = self._create_DH()     
        
        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_ur3', 
                            link1 = 'shoulder_ur3', 
                            link2 = 'upperarm_ur3', 
                            link3 = 'forearm_ur3', 
                            link4 = 'wrist1_ur3', 
                            link5 = 'wrist2_ur3', 
                            link6 = 'wrist3_ur3')
        
        # A joint config and the 3D object transforms to match that config
        qtest = [0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(0,0,0),                                                     
                            spb.transl(0,0,0.15239) @ spb.trotz(pi), 
                            spb.transl(0,-0.12,0.1524) @ spb.trotz(pi),
                            spb.transl(0,-0.027115,0.39583) @ spb.trotz(pi), 
                            spb.transl(0,-0.027316,0.60903) @ spb.rpy2tr(0,-pi/2,pi, order = 'xyz'),
                            spb.transl(0.000389,-0.11253,0.60902) @ spb.rpy2tr(0,-pi/2,pi, order= 'xyz'),
                            spb.transl(-0.083765,-0.11333,0.61096) @ spb.trotz(pi)]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'UR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest
        
    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
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
        self.base = SE3(0.5,0.5,0)
        self.add_to_env(env)
             
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)
        # env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r = UR3()
    r.test()

    
