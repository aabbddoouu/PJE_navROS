# PJE_navROS
Source files for PJE09 navROS.
The main branch (kinetic) contains the whole directory. See the 2nd branch for changed files only.

In this 1st iteration, we are using the Husky (see [Husky github page](https://github.com/husky/husky)) robot to conduct our comparative study of 3 SLAM Methods :
- Gmapping
- Hector slam
- Karto Slam

The bash script **script.sh** automates the latter commands.
Make sure to set it as an executable to run it.

## Launch gazebo
roslaunch husky_gazebo husky_playpen.launch ur5_enabled:=false laser_enabled:=true

## Launch SLAM method && move base
- roslaunch husky_navigation gmapping_demo.launch 


- roslaunch husky_navigation hector_demo.launch


- roslaunch husky_navigation karto_slam.launch  [Version used](https://github.com/nkuwenjian/slam_karto)  ()


 

## Launch Rviz for visualization
roslaunch husky_viz view_robot.launch 

## Save Map
rosrun map_server map_saver -f ($ map_name)

## How2Add 2laser scans
### URDF : Robot Model
```
<xacro:arg name="laser_enabled" default="false" />
<xacro:arg name="laser1_xyz" default="$(optenv HUSKY_LMS1XX_XYZ 0.2206 0.0 0.00635)" />
<xacro:arg name="laser1_rpy" default="$(optenv HUSKY_LMS1XX_RPY 0.0 0.0 0.0)" />
<xacro:arg name="laser2_xyz" default="$(optenv HUSKY_LMS1XX_XYZ -0.2206 0.0 0.00635)" />
<xacro:arg name="laser2_rpy" default="$(optenv HUSKY_LMS1XX_RPY 0.0 0.0 3.14)" />

   <xacro:if value="$(arg laser_enabled)">

    <xacro:if value="$(arg 2_laser_enabled)">

      <sick_lms1xx_mount prefix="base1"/>

      <sick_lms1xx frame="base1_laser" topic="scan1" robot_namespace="$(arg robot_namespace)" update_rate="20"
               min_angle="-2.35619" max_angle="2.35619" min_range="0.1" max_range="9.0"/>

      <joint name="laser1_mount_joint" type="fixed">
        <origin xyz="$(arg laser1_xyz)" rpy="$(arg laser1_rpy)" />
        <parent link="top_plate_link" />
        <child link="base1_laser_mount" />
      </joint>

      <sick_lms1xx_mount prefix="base2"/>

      <sick_lms1xx frame="base2_laser" topic="scan2" robot_namespace="$(arg robot_namespace)" update_rate="20"
               min_angle="-2.35619" max_angle="2.35619" min_range="0.1" max_range="9.0"/>

      <joint name="laser2_mount_joint" type="fixed">
        <origin xyz="$(arg laser2_xyz)" rpy="$(arg laser2_rpy)" />
        <parent link="top_plate_link" />
        <child link="base2_laser_mount" />
      </joint>

    </xacro:if>
<!--  </xacro:if> -->

    <xacro:unless value="$(arg 2_laser_enabled)">

      <sick_lms1xx_mount prefix="base"/>

      <sick_lms1xx frame="base_laser" topic="scan" robot_namespace="$(arg robot_namespace)" update_rate="20"
               min_angle="-2.35619" max_angle="2.35619" min_range="0.1" max_range="9.0"/>

      <joint name="laser_mount_joint" type="fixed">
        <origin xyz="$(arg laser1_xyz)" rpy="$(arg laser1_rpy)" />
        <parent link="top_plate_link" />
        <child link="base_laser_mount" />
      </joint>

    </xacro:unless>

  </xacro:if>
```

### Merge 2scans into 1 w/ ira_laser_tools 

roslaunch ira_laser_tools laserscan_multi_merger.launch 


## Control robot with xbox controller
# Commands
 
- sudo xboxdrv --device-by-id 2f24:0091 --type xbox360 --device-name 'EasySMX-9101' --silent
    - it opens a serial port for the controller : typ. **/dev/input/js1**
    - for first time use : make the device read/write-able eg. *chmod 777 /dev/input/js1*
- bring up the robot
- rosparam set /joystick/dev "/dev/input/js1"
- roslaunch teleop_twist_joy teleop.launch
    - service that converts controller data (**Joy.msg**) to **Twist.msg** and publishes it to the topic **/cmd_vel**


# Troubleshooting 

- rostopic echo /teleop_velocity_smooer/raw_cmd_vel
    - Check if the turtlebot_teleop is working
- rostopic echo joy
    - Check if ros is receiving controller data

# Bugs/Problems
- the merged scan goes beyond laser range defined in robot description
  - cause : 'inf' scanned points gets reduced to *range_max* +1 defined in laser\_scan\_multi\_merger
  - solution : set *range_max* to 9.0 and set scan max distance in each SLAM method to 8.1 (>8.0)

- Karto's Karto node (Karto.cpp) has a problem in its "Validate" function when merging scans :
  - cause : GetNumberOfRangeReadings() is off by 1  
  - solution :
  ```
    kt_bool LaserRangeFinder::Validate(SensorData* pSensorData)
  {
    LaserRangeScan* pLaserRangeScan = dynamic_cast<LaserRangeScan*>(pSensorData);

    // verify number of range readings in LaserRangeScan matches the number of expected range readings
    if (abs(pLaserRangeScan->GetNumberOfRangeReadings() - GetNumberOfRangeReadings())<=1){
      return true;
    }
    /*Old code 
    if (pLaserRangeScan->GetNumberOfRangeReadings() != GetNumberOfRangeReadings())
    {
      std::cout << "LaserRangeScan contains " << pLaserRangeScan->GetNumberOfRangeReadings()
                << " range readings, expected " << GetNumberOfRangeReadings() << std::endl;
      return false;
    }
    */
  ```
- Global cost map does "du n'importe quoi" when it is resized 
  - cause : old version with a wrong commit : [Github issue](https://github.com/ros-planning/navigation/issues/959)
  - solution : update to new version 

- Hector Slam *teleports* map when walking in a straight corridor (myworld)
  - cause : ??? maybe it doesn't use odometry ? But the fram odom is passed as an argument when calling the node ...
  - solution : ???

- When turning (slow and no linear speed), Karto slam (with loop closing enabled) sometimes shifts the map *
  - cause : odometry ?? But gmapping uses odometry and doesnt suffer from this problem (or as much)
  - solution : ???
