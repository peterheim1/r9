from ros2_launch_util import *

def launch(ld, argv):
    # Launch a robot state publisher using URDF
    #     robbie/urdf/robbie.urdf
    #add_robot_state_publisher_urdf(
        #ld, "robbie", "urdf", "robbie.urdf")
    
    # Or use xacro (which will be converted to urdf)
    #     robbie/urdf/r2.xarco
    add_robot_state_publisher_xacro(
        ld, "robbie", "urdf", "r2.xarco")
    
    # Include a launch file: other_package/launch/other_package.py
    #add_launch_file(ld, "other_package", "other_package.py")
    
    # Add a static transform publisher with an x offset
    add_static_transform_publisher(ld, "world", "map", x=1.0)


    # Easily launch the my_node node within the my_package package
    #add_node("my_package", "my_node", args=nodeArgs)
    
    # Find a file in the package's share directory:
    #     my_package/config/info.cfg
    #configFile = find_share_file("my_package", "config", "info.cfg")
    
    # Find an executable within a package:
    #execPath = find_executable("my_package", "my_executable")
    
    # Convert an xacro file to a URDF file, returns the path to
    # the resulting URDF file as a temporary file
    # (e.g., /tmp/r2.xarco_siu_qyii)
    urdfFile = xacro_to_urdf(
        "robbie", "urdf", "r2.xarco")
    
    # Convert an xacro file to a specific URDF file
    urdfFile = xacro_to_urdf(
        "robbie", "urdf", "r2.xarco",
        urdf_file="/tmp/robbie.urdf")
