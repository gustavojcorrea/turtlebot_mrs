<launch>

  <arg name="scot_name"/>

  <group ns="/$(arg scot_name)">
    
    <arg name="cmd_vel_topic" default="mobile_base/commands/velocity" />
    <!--<arg name="cmd_vel_topic" default="mobile_base/commands/motor_power" -->
    <arg name="odom_topic" default="odom" />
    
    <node pkg="move_base" type="move_base" respawn="false" name="move_base_node" output="screen">
        <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
        <param name="base_global_planner" value="global_planner/GlobalPlanner" />

        <rosparam file="$(find mr_scot)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find mr_scot)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find mr_scot)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/move_base_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/dwa_local_planner_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/global_planner_params.yaml" command="load" />

        <param name="~/global_costmap/robot_base_frame" value="$(arg scot_name)/base_footprint" />
        <param name="~/global_costmap/global_frame" value="$(arg scot_name)/map" />
        <param name="~/local_costmap/robot_base_frame" value="$(arg scot_name)/base_footprint" />
        <param name="~/local_costmap/global_frame" value="$(arg scot_name)/odom" />

        <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
        <remap from="odom" to="$(arg odom_topic)"/>
	
    </node>
  </group>
</launch>

<!--
<launch>

  <arg name="scot_name"/>

  <group ns="/$(arg scot_name)">
    
    <arg name="cmd_vel_topic" default="cmd_vel" />
    <arg name="odom_topic" default="odom" />
    <node pkg="move_base" type="move_base" respawn="false" name="move_base_node" output="screen">
        <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
        <param name="base_global_planner" value="global_planner/GlobalPlanner" />

        <rosparam file="$(find mr_scot)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find mr_scot)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find mr_scot)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/move_base_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/dwa_local_planner_params.yaml" command="load" />
        <rosparam file="$(find mr_scot)/param/global_planner_params.yaml" command="load" />

        <param name="~/global_costmap/robot_base_frame" value="$(arg scot_name)/base_footprint" />
        <param name="~/global_costmap/global_frame" value="$(arg scot_name)/map" />
        <param name="~/local_costmap/robot_base_frame" value="$(arg scot_name)/base_footprint" />
        <param name="~/local_costmap/global_frame" value="$(arg scot_name)/odom" />

        <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
        <remap from="odom" to="$(arg odom_topic)"/>
    </node>
  </group>
</launch>

-->
