#ifndef GAMMA_SIMULATOR_NODE_H_
#define GAMMA_SIMULATOR_NODE_H_

//C++
#include <gamma_simulator/agent_info.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <unordered_map>
#include <limits>
#include <thread>
#include <mutex>

//ROS
#include <gamma_simulator/Agent.h>
#include <gamma_simulator/GammaParams.h>
#include <gamma_simulator/RVO.h>
#include <gamma_simulator/RVOSimulator.h>
#include <gamma_simulator/AgentStates.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gamma_simulator/pid.h>
#include <nav_msgs/Path.h>


ros::Publisher robot_vel_pub_;
ros::Publisher agent_states_pub_;
ros::Publisher obstacles_viz_pub_;
ros::Publisher waypoint_pub_;
ros::Subscriber global_plan_sub_;

ros::Timer robot_odom_timer_;
std::thread robot_sim_thread_;
ros::ServiceServer reset_gamma_serv_;
tf2_ros::Buffer tf_buf;
geometry_msgs::Pose global_plan_waypoint;
geometry_msgs::Twist robot_twist_cmd;
double waypoint_distance = 0.9;
std::mutex valid_mutex;

RVO::RVOSimulator *gamma_sim_ = nullptr;

//GAMMA parameters
float timeStep;
float neighborDist = 8.0;
int maxNeighbors = 25;
float timeHorizon = 4.0;
float timeHorizonObst = 0.3;
float radius = 0.25;
float maxSpeed = 1.0;
float rate = 60;
double heading_filter_const = 0.92;
double waypoint_filter_const = 0.92;
bool use_polygon = true;
bool consider_kinematics = true;
bool use_dynamic_resp = true;
bool use_dynamic_att = true;
bool simulate_robot = true;
double heading_filter_time_const = 0.2;
double waypoint_filter_time_const = 0.5;
int seed = 1;
double robot_radius = 0.4;

//GAMMA scenario parameters
std::string scenario_name = "hospital";
std::string waypoints_file;
std::string obstacles_file;
std::string agents_file;

//PID Parameters
double steering_p;
double steering_i;
double steering_d;
double steering_f;
double steering_output_ramp = 0;
double steering_output_desc = 0;
double steering_max_output = 0;
double steering_min_output = 0;
double steering_max_i_output = 0;

double velocity_p;
double velocity_i;
double velocity_d;
double velocity_f;
double velocity_output_ramp = 0;
double velocity_output_desc = 0;
double velocity_max_output = 0;
double velocity_min_output = 0;
double velocity_max_i_output = 0;

double pid_freq = 10;

PID steering_pid;
PID velocity_pid;

/**
 * Robot's odometry in the map frame
 **/
AgentInfo robot_info_;

std::vector<std::vector<RVO::Vector2>> obstacles_;
std::vector<AgentInfo> agents_;
std::vector<AgentInfo> agents_initial_;
std::unordered_map<std::string, RVO::Vector2> waypoints_;
int robot_agent_id_;
gamma_simulator::AgentStates agent_states_;

#endif
