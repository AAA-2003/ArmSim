# ArmSim
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
