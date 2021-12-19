Para que el robot se mueva al waypoint dado con 2d nav goal sólo se ha cambiado el launch a lo siguiente:
(no hace falta tocar nada en el nodo diff_drive_goto goal):
````
<launch>

  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="0.0"/>
  <arg name="y_pos" default="0.0"/>
  <arg name="z_pos" default="0.0"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/empty.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />




  <arg name="ticks_per_meter" value="10000" />
  <arg name="wheel_separation" value="0.2" />

  <arg name="urdf_file" default="$(find diff_drive)/urdf/diff_drive_description.xacro" />
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg urdf_file)" />

  <!-- Publish the robot state -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher">
    <param name="publish_frequency" value="10.0"/>
  </node>

  <!-- Provide simulated control of the robot joint angles -->
  <node name="joint_state_publisher" pkg="joint_state_publisher"
        type="joint_state_publisher">
    <param name="use_gui" value="False" />
    <param name="rate" value="10.0"/>
  </node>

  <!--node name="controller" pkg="diff_drive" type="diff_drive_controller"
        output="screen">
    <rosparam subst_value="true">
      ticks_per_meter: $(arg ticks_per_meter)
      wheel_separation: $(arg wheel_separation)
      max_motor_speed: 3000
      timeout: 1.0
    </rosparam>

    <remap from="cmd_vel" to="/robot/cmd_vel" />

  </node-->
  
  <!--node name="odom_publisher" pkg="diff_drive" type="diff_drive_odometry"
        output="screen">
    <rosparam subst_value="true">
      ticks_per_meter: $(arg ticks_per_meter)
      wheel_separation: $(arg wheel_separation)
    </rosparam>
  </node-->

  <!--node name="robot" pkg="diff_drive" type="diff_drive_mock_robot"
        output="screen">
    <remap from="~lwheel_desired_rate" to="lwheel_desired_rate" />
    <remap from="~rwheel_desired_rate" to="rwheel_desired_rate" />
    <remap from="~lwheel_ticks" to="lwheel_ticks" />
    <remap from="~rwheel_ticks" to="rwheel_ticks" />
  </node-->

  <node name="diff_drive_go_to_goal" pkg="diff_drive"
        type="diff_drive_go_to_goal" output="screen">
    <param name="~rate" value="20" />
    <param name="~kP" value="0.5" />
    <param name="~kA" value="1.0" />
    <param name="~kB" value="-0.8" />
    <param name="~max_linear_speed" value="0.2" />
    <param name="~min_linear_speed" value="0.05" />
    <param name="~max_angular_speed" value="0.7" />
    <param name="~min_angular_speed" value="0.1" />
    <param name="~linear_tolerance" value="0.01" />
    <param name="~angular_tolerance" value="0.04" />
    <param name="~forwardMovementOnly" value="false" />
    <!--
    <remap from="cmd_vel" to="/robot/cmd_vel" />
    -->
  </node>
  
  
  <node name="rviz" pkg="rviz" type="rviz"
        args="-d $(find diff_drive)/config/view.rviz" />
 
  

</launch>
````
