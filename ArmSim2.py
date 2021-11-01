# -*- coding: utf-8 -*-
"""
Created on Sat Apr 26 00:23:21 2021
@author: Ayngaran Adalarasu
ROBOTIC ARM SIMULATOR V2.0.0
written in: Python 3.8
This is meant to be imported into a seperate program and used as a module
"""

############importing necessary packages##############
import numpy as np #for trig, pi & matrix addition functions

import matplotlib.pyplot as plt #for displaying arm's and it's link's final position
from mpl_toolkits.mplot3d import axes3d #importing 3D axises
import copy #for deep copying lists


#### link object description class ####
#Structural & functional unit of robotic arm class
#!!!DO NOT Instantiate, access or use directly!!! ALWAYS USE ARM CLASS
class link:
    #variables describing nature of link
    length = 0 #Z translation
    offset = (0,0) #(X, Y Translation) of set of joints rotation in plane perpendicular to Z axis
    r_axis = ()  #specifies possible axeses along which the link can rotate | (1,2,3) << (x,y,z) axeses
    
    #initalises new link object
    #length of link is madatorry parameter, offset is optional
    #if want independent rotational actuator >> length = 0
    def __init__(self, length, offset=(0,0), r=(1,1,1)):
        self.length = length
        self.offset = offset 
        
        t = []
        for i in range(3):
            if r[i]:
                t.append(i+1)
        self.r_axis = tuple(t)
    
    #generates and return homogenous transformation matrix
    #input: rotation angles along (x,y,z), !!!all angles are in radians!!!
    #returns: 4x4 homogenous transformation matrix account Z-Y-X Euler roation >> translation along length of link
    #Computes WRT to Z-Y-X euler angles <<equivalent>> X-Y-Z euler angles 
    #also accounts for translation due to lateral offset of link (self.offset)
    def get_trans_matrix(self, theta):
        #cos function of matrix
        def c(t):
            return np.cos(t)
        #sin function of matrix 
        def s(t):
            return np.sin(t)
        
        t = list(theta)
        rx = ry = rz = 0
        
        if 1 in self.r_axis:
            rx = t.pop(0)
        if 2 in self.r_axis:
            ry = t.pop(0)
        if 3 in self.r_axis:
            rz = t.pop(0)
            
        #Homogenous transformation matrix
        T_matrix = np.array([
            [    c(rz)*c(ry),     c(rz)*s(ry)*s(rx) - s(rz)*c(rx),      c(rz)*s(ry)*c(rx) + s(rz)*s(rx),  self.offset[0]],
            [    s(rz)*c(ry),     s(rz)*s(ry)*s(rx) + c(rz)*c(rx),      s(rz)*s(ry)*c(rx) - c(rz)*s(rx),  self.offset[1]],
            [     -1*s(ry)   ,                 c(ry)*s(rx)       ,                  c(ry)*c(rx)        ,  self.length   ],
            [         0      ,                       0           ,                        0            ,      1         ]])
        
        #T_matrix = Rzyx + Translation matrix 
        #Rzyx = Rz(rz).Ry(ry).Rx(rx)
        #Rotation WRT to Z-Y-X Euler angles 
        
        
        return(T_matrix)

##### END OF LINK CLASS   ######



### Robotic arm object modelling class ###
#Creates virtual model of robotic arm based on input characteristics given
#User to Instantiate this class for making arm models
#constructs arm using link() objects
#Functions:
#   *models arm parameters
#   *performs forward kinematics
#   *displays arm graphically using matplotlib
#User Callable methods:
#  + Forward Kinematics:
#    * get_end_pos()
#    * get_arm_pos()
#    * display_arm(prt_label = <True or False>) 
#  + Inverse Kinematics
#    * hill_climb([<end pos>], start=[<array of angles>])
#    * plot_trajectory([<start_pos>], [<end_pos>], display=<True or False>)
#    * plot_path([[coordinates]...] , display=<True or False>)
#  + Getters and setters
#    * get_DOF()
#    * home()  << sets arms angles to home angles
#    * get_angles()
#    * get_global_offset()
#    * set_global_offset([<offset>])
#    * get_work_area()
#    * set_work_area([<-x, +x, -y, +y>])
#    * get_display_limits()
#    * set_display_limits(<'x' or 'y' or 'z'>, [<upper, lower limit>])
#    * get_name()
#    * set_name(<name>)
#    * get_path_res()
#    * set_path_res(<resolution>)
#    * set_step_size([<steps>])  << for hill climbing (inverse kinematics)
#    * get_step_size()  << for hill climbing (inverse kinematics)
#    * set_err_radius(<radius>) << for hill climbing (inverse kinematics)
#    * get_err_radius()  << for hill climbing (inverse kinematics)
#    * set_itr_limit(<limit>)  << for hill climbing (inverse kinematics)
#    * get_itr_limit()  << for hill climbing (inverse kinematics)


class arm:
    #variables describing arm characteristics 
    name = "Robotic arm"
    global_offset = (0,0,0) #offset of entire arm, i.e offset of base of arm from GCA
    base_work_area = (5, 31, 22, 22)  #distances of work area boundary from global coordinate axes (-x, +x, -y, +y)
    home_pos = []
    
    #Variables describing display characteristics
    display_x_limits = (-5, 20) #x limits of output display, CAN BE MODIFIED BY USER
    display_y_limits = (-20, 20) #y limits of output display, CAN BE MODIFIED BY USER
    display_z_limits = (0, 20) #z limits of output display, CAN BE MODIFIED BY USER
    
    #INVERSE KINEMATICS SETTINGS (hill climbing)
    #step_size = (0, 10, -10, 1, -1) #possible steps(in degrees) arm can take to reach target
    step_size = (0, 10, -10, 0.1, -0.1) #possible steps(in degrees) arm can take to reach target
    err_radius = 0.1 #margin of error radius from target, within which hill climbing tries to bring arm end effector to
    itr_overflow = 200 #max no. of iterations hill climbing attempts before which it gives up
    
    #Path plotter settings
    path_res = 0.1 #smallest unit resolution the greatest difference WRT to a axis between 2 points is divided into
    
    #Variables for internal calculation 
    arm_links = []
    link_angles = [] #stores last used link angles of arm | updated when get_end_pos() or get_arm_pos( is called)
    DOF = 0 #total degrees of freedom in arm
    
    link_name_key = {}
    
    functional = False
    
    
    def __init__(self, arm_file):
        #NEW MOD to fix multi object creation problem
        self.arm_links = []
        self.link_angles = []
        self.link_name_key = {}
        if not self.import_arm(arm_file):
            print("arm modelling from ", arm_file, "was NOT sucessful, please try again")
        else:
            self.functional =True
    
    #Converts all elements of any 2D array to degrees
    #works even on ragged arrays; np.radians DOES NOT WORK on ragged arrays
    def deg2rad ( self, deg ) :
        radL = []
        for degL in deg:
            radL.append(np.radians(degL))
        return radL
    
    
    #Computes end effector position of arm
    #takes 2D array containing rotation angles [rx, ry, rz] for each link in robotic arm
    #input anles are in degrees
    #returns: final (x,y,z) posiiton of end effector as array
    def get_end_pos(self, t_in):
        if not self.functional:
            return False
        
        self.link_angles = copy.deepcopy(t_in)
        thetas = self.deg2rad(t_in)  #converting to radians
        
        arm_pose = self.arm_links[0].get_trans_matrix(thetas[0])
        
        #taking dot product of poses consecutively to get net arm pose
        for i in range(1, len(self.arm_links)):
            arm_pose = arm_pose @ self.arm_links[i].get_trans_matrix(thetas[i]) #expanding pose by getting dot product
            
        end_pos = arm_pose @ [0,0,0,1] #getting end position in term of base coordinate axes, by taking dot product
        
        return(np.round(end_pos[:3], 4) + self.global_offset) #rounding of to 4 decimal places and returning
    
    
    
    #computes position of all joints of arm
    #takes 2D array containing rotation angles [rx, ry, rz] for each link in robotic arm
    #all input angles are in degrees
    #returns: 2D arrray containing (x,y,z) position of each link of arm
    def get_arm_pos(self, t_in):
        if not self.functional:
            return False
        self.link_angles = copy.deepcopy(t_in)
        
        thetas = self.deg2rad(t_in)  #converting to radians
        
        arm_pos = []
        arm_pose = self.arm_links[0].get_trans_matrix(thetas[0])
        arm_pos.append((arm_pose @ [0,0,0,1])[0:3])
        for i in range(1, len(self.arm_links)):
            arm_pose = arm_pose @ self.arm_links[i].get_trans_matrix(thetas[i])
            arm_pos.append((arm_pose @ [0,0,0,1])[0:3])
            
        return(np.round(arm_pos, 4) + self.global_offset)
    
    
    ####INVERSE KINEMATICS #####
    
    def hill_climb(self, target_pos, start = home_pos, steps=False, err_radius=False, itr_limit = False):
        #calculates euclidean distance between 2 arrays
        def euclid_dist(a, b):
            out = 0
            for i in range(len(a)):
                out += (a[i]-b[i])**2
            return np.sqrt(out)
        
        if not steps:
            steps = self.step_size
        if not err_radius:
            err_radius = self.err_radius 
        if not itr_limit:
            itr_limit = self.itr_overflow
            
            
        ''' To give corresponding angles in parent itself
        if start == "home":
            a = self.home_pos[:]
        elif start == "prev":
            a = self.link_angles
        else:
            a = start'''
        
        a = start
            
        best_dist = euclid_dist(self.get_end_pos(a), target_pos)
        itr = 0
        
        while best_dist >= err_radius and itr <= itr_limit:
            for j in range(len(a)):
                for t in range(len(a[j])):
                    best_step = steps[0]
                    for step in steps:
                        a[j][t] += step
                        dist = euclid_dist(self.get_end_pos(a), target_pos)
                        if dist < best_dist:
                            best_dist = dist
                            best_step = step
                        a[j][t] -= step
                    a[j][t] += best_step
            itr += 1
           # print(a)
        if itr >= itr_limit:
            print("Iteration limit Overfow!! Hill Climbing has failed")
            print("distance from target =", best_dist)
            return False
        return a
        
    
    def plot_trajectory(self, start_pos, end_pos, display=False, res =path_res):
        p_out = []
        steps = int(max(abs(end_pos[0] - start_pos[0]), abs(end_pos[1] - start_pos[1]), abs(end_pos[2]-start_pos[2]))*(1/res))
        x_gradient = (end_pos[0] - start_pos[0])/(1.0*steps)
        y_gradient = (end_pos[1] - start_pos[1])/(1.0*steps)
        z_gradient = (end_pos[2] - start_pos[2])/(1.0*steps)
        # print steps, x_gradient, y_gradient
        prev_out = self.home_pos
        trajectory_xyz = []
        for i in range(steps):
            x = start_pos[0]+(i*x_gradient)
            y = start_pos[1] + (i*y_gradient)
            z = start_pos[2] + (i*z_gradient)
            out = self.hill_climb([x,y,z], start=prev_out)
            #pp_out += "{"+str(out[0][0])+", "+str(out[1][0]) + ", "+str(out[2][0])+ ", "+str(out[3][0])+"}, \n"
            prev_out = out[:]
            p_out.append(copy.deepcopy(out))
            trajectory_xyz.append(self.get_end_pos(out))
        if display:
            self.display_arm(trajectory = trajectory_xyz)
        return (p_out, trajectory_xyz)
        
    def plot_path(self, points, display=True, res = path_res):
        trajectory_xyz = []
        p_out = []
        for i in range(1, len(points)):
            (p,t) = self.plot_trajectory(points[i-1], points[i], display=False, res = res)
            trajectory_xyz += t
            p_out += p
        if display:
            self.display_arm(trajectory = trajectory_xyz)
        return p_out, trajectory_xyz
    
    
    #display complete arm graphically, after performing forward kinematics
    #graphical display using matplotlib
    #!!!To be called only after performing forward kinematics atleas once!!! i.e after calling get_end_pos() or get_arm_pos()
    def display_arm(self, prt_label = True, trajectory=[]):
        if not self.functional:
            return False
        self.fig = plt.figure(figsize=(9,9))
        link_coord = self.get_arm_pos(self.link_angles)
        #self.ax = plt.axes(projection='3d')
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.ax.clear()
        self.ax.set(xlim=self.display_x_limits, ylim = self.display_y_limits, zlim=self.display_z_limits)
        self.ax.set(xlabel='X axis', ylabel='Y axis', zlabel='Z axis')
        
        self.ax.set_box_aspect((1,1.6,1))
        
        x = [self.global_offset[0]]
        y = [self.global_offset[1]]
        z = [self.global_offset[2]]
        for link in link_coord:
            x.append(link[0])
            y.append(link[1])
            z.append(link[2])
            
        self.ax.plot3D(x,y,z, linewidth=5)
        self.ax.scatter(x[1:-1],y[1:-1],z[1:-1], s=40, c="c", alpha=1)
        self.ax.scatter(x[-1], y[-1], z[-1], s=40, c="red", alpha=1)
        
        #labelling additional info
        if prt_label:
            #labelling rotation angle of each joint
            for i,a in enumerate(self.link_angles):
                def round_off(x):
                    return(round(x, 2))
                self.ax.text(x[i+1], y[i+1]+0.2, z[i+1] , str(list(map(round_off, a))))
            
            #drawing base boundaries
            self.ax.plot3D([-1*self.base_work_area[0], -1*self.base_work_area[0]], [-1*self.base_work_area[2], self.base_work_area[3]], [0,0], linestyle="dashed", color="red", alpha=0.5)
            self.ax.plot3D([self.base_work_area[1], self.base_work_area[1]], [-1*self.base_work_area[2], self.base_work_area[3]], [0,0], linestyle="dashed", color="red", alpha=0.5)
            self.ax.plot3D([-1*self.base_work_area[0], self.base_work_area[1]], [-1*self.base_work_area[2], -1*self.base_work_area[2]], [0,0], linestyle="dashed", color="red", alpha=0.5)
            self.ax.plot3D([-1*self.base_work_area[0], self.base_work_area[1]], [self.base_work_area[3], self.base_work_area[3]], [0,0], linestyle="dashed", color="red", alpha=0.5)
            #drawing lines through GCA
            self.ax.plot3D([-1*self.base_work_area[0], self.base_work_area[1]], (0,0), (0,0), linestyle="dashed", color="grey", alpha=0.7)
            self.ax.plot3D((0,0), [-1*self.base_work_area[2], self.base_work_area[3]], (0,0), linestyle="dashed", color="grey", alpha=0.7)
            
            #labelling end effector position
            self.ax.scatter(x[-1], y[-1], 0, s=40, c="purple", alpha=0.6, marker="x")
            self.ax.text(x[-1], y[-1]+0.3, 0, str(np.round((x[-1], y[-1]), 2)), c="grey", alpha=0.8)
            self.ax.plot3D([x[-1], x[-1]], [y[-1], y[-1]], [0, z[-1]], linestyle="dashed", color="green", alpha=0.5)
            self.ax.text(x[-1], y[-1]+0.2, z[-1]/2, "Z: "+str(np.round(z[-1], 2)), c="grey", alpha=0.8)
        
        #plotting trajectory, when called from slef.plot_trajectory(display = True)
        if len(trajectory)  > 0:
            for pt in trajectory:
                self.ax.scatter(pt[0], pt[1], pt[2], s=1, c='purple', alpha=0.85)
                
                
        plt.title(self.name)
        plt.show()
        return(True)
    
    def home(self):
        self.link_angles = self.home_pos[:]
        
    ##Setters & Getters
    #retrun True for sucess, False for error
    
    #returns current angle configuration of each link 
    def get_angles(self):
        return self.link_angles
    
    #returns total degrees of freedom in arm
    def get_DOF(self):
        return self.DOF
    
    
    def set_step_size(self, n):
        self.step_size = n
        return True
        
    def get_step_size(self):
        return self.step_size
    
    def set_err_radius(self, r):
        self.err_radius = r
        return True
    
    def get_err_radius(self):
        return self.err_radius
    
    def set_itr_limit(self, l):
        self.itr_overflow = l
        return True 
        
    def get_itr_limit(self):
        return self.itr_overflow
        
    #global offset setter & getter 
    def set_global_offset(self, n):
        if len(n) != 3:
            return False 
        self.global_offset = n
        return True 
    
    def get_global_offset(self):
        return self.global_offset
    
    #work_area setter & getter 
    def set_work_area(self, n):
        if len(n) != 4:
            return False 
        self.work_area = n
        return True 
    
    def get_work_area(self):
        return self.work_area
    
    def get_path_res(self):
        return self.path_res
    
    def set_path_res(self, r):
        self.path_res = r
        return True
    
    
    #for setting display limit values
    #input: give axes ('x' or 'y' or 'z') and value of lower and upper limit as tuple: (lower, upper)
    def set_display_limit(self, axes, value):
        if len(value) != 2:
            return(False)#returning false if input value is error
        
        if axes == 'x':
            self.display_x_limits = value
        elif axes == 'y':
            self.display_y_limits = value
        elif axes == 'z':
            self.display_z_limits = value
        else:
            return(False) #retrun false due to wrong axes input
        return(True)# returning true if everything goes fine 
    
    #returns dsiplay limit of (x, y, z)
    def get_display_limits(self):
        return((self.display_x_limits, self.display_y_limits,self.display_z_limits))
    
    #Setter getter for self.name
    def set_name(self, n):
        self.name = n
        return(True)
    
    def get_name(self):
        return self.name 
    
    
    #####INPUT PARSERS: inputting arm data from .arm file ######
    
    def import_arm(self, arm_file):
        arm_f = open(arm_file, "r")
        
        link_data = False
        for line_no, line in enumerate(arm_f):
            in_line = line.strip().strip('(').strip(')')
            if len(in_line) == 0:
                continue
            elif in_line == "LINKS:":
                link_data = True
            elif in_line == "END LINKS":
                link_data = False
            elif link_data:
                param = in_line.split(";")
                if len(param) < 2:
                    print("InputError: in line#", line_no+1, "Inadequate link description given !!!")
                    return False
                #for mandatory inputs
                if param[0].strip() in self.link_name_key.keys():
                    print("NameError: in line#", line_no+1, ">> name", param[0], " is already used!!")
                    return False
                self.link_name_key[param[0].strip()] = len(self.arm_links)
                
                if "L:" not in param[1]:
                    print("InputError: in line#", line_no+1, "L:, Length of link is a mandatory input!!!")
                    return False
                l = float(param[1].strip().strip("L:"))
                
                #for auxillary inputs
                face_off = (0,0)
                r_a = [1,1,1]
                for i in range(2, len(param)):
                    if "OFF:" in param[i]:
                        face_off = list(map(float, param[i].split(":")[1].strip().strip("(").strip(")").split(",")))
                        if len(face_off) != 2:
                            print("InputError: in line#", line_no+1, "OFF: offset takes in only 2 inputs")
                            return False
                    elif "R" in param[i]:
                        r_a = [0,0,0]
                        if 'x' in param[i]:
                            r_a[0] = 1
                            self.DOF += 1
                        if 'y' in param[i]:
                            r_a[1] = 1
                            self.DOF += 1
                        if 'z' in param[i]:
                            r_a[2] = 1
                            self.DOF += 1
                    else:
                        print("InputError: in line#", line_no+1, "unidentified input parameter given: ", param[i])
                        return False
                
                self.arm_links.append(link(l, offset=(face_off), r=tuple(r_a)))  #adding the link to the arm 
                    
            
            elif "NAME:" in in_line:
                self.name = in_line.split(":")[1].strip()
            elif "GLOBAL_OFFSET" in in_line:
                off = list(map(int, in_line.split(":")[1].strip().strip("(").strip(")").split(",")))
                if len(off) != 3:
                    print("InputError: in line#", line_no+1, "global offset requires 3 parameters")
                    return False
                self.global_offset = off
                
                
                ##NOT DONE
            elif "HOME_POS" in in_line:
                l = in_line.split(":")[1].strip().strip('(').strip(')').split('(')
                for j in l:
                    t = j.strip().strip(',').strip(')').split(',')
                    for q in range(len(t)):
                        t[q] = t[q].strip(')')
                    self.home_pos.append( list( map(int, t)) )
                self.link_angles = self.home_pos[:]
            else:
                print("InputError: in line#", line_no+1, "unidentified data found ", in_line)
                return False
            
        #final validation
        error = False
        if len(self.arm_links) < 1:
            print("Error: Atleast more than 1 link description is required")
            error = True
        if link_data:
            print("Error: Description of all links is not terminated, END LINKS is missing !!!")
            error = True
        
        if error:
            return False
        return True 
        
        
##### END OF ARM CLASS   ######
        

##### MAIN - FOR TEST PURPOSES ONLY  ######

#!!! To be imported and used in a seperate program !!!
#import ArmSim2.py

##### END OF PROGRAM :)  ####