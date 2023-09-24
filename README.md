# Gazebo_Skid_Steer_Plugin
Skid Steer plugin for a 4 wheel skid steer drive robot

#This plugin has been modified from the Diff-drive-plugin

#make sure to edit and add this in the bashscript for ease
```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:path/to/plugin/skid_steer_drive_plugin/build
```
#Sample usage
```
  <gazebo>
      <plugin name="skid_steer_controller" filename="libskid_steer_drive_plugin.so">
        
        <front_left_joint>...</front_left_joint>
        <front_right_joint>...</front_right_joint>
        <back_left_joint>...</back_left_joint>
        <back_right_joint>...</back_right_joint>

        <robot_base_frame>base_footprint</robot_base_frame>

        <num_wheel>4</num_wheel>
        <wheel_separation>...</wheel_separation>
        <wheel_separation_length>...</wheel_separation_length> 
        <wheel_radius>...</wheel_radius>

        <max_wheel_acceleration>10.0</max_wheel_acceleration> 
        <max_wheel_torque>100.0</max_wheel_torque>

        <update_rate>100.0</update_rate> 

        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>true</publish_wheel_tf>

        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>chassis</robot_base_frame>
      </plugin>
    </gazebo>
```
