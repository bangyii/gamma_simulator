#ifndef GAMMA_SIMULATOR_NODE_H_
#define GAMMA_SIMULATOR_NODE_H_

//C++
#include <vector>
#include <string>
#include <cstdlib>
#include <unordered_map>
#include <limits>
#include <thread>

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

class AgentInfo
{
public:
    std::vector<RVO::Vector2> waypoints_;
    int current_waypoint_ = 0; //Start with 1 because prefVel will decrement this on initialization
    int id_ = -1;
    nav_msgs::Odometry odom_, last_odom_;
    double distance_threshold_ = 0.3;
    double odom_update_thresh_ = 0.2;
    double heading_ = RVO::RVO_ERROR;
    double heading_filter_const_ = 0.90;
    double waypoint_filter_const_ = 0.90;
    RVO::Vector2 cur_waypoint_offset_ = RVO::Vector2(0, 0);
    RVO::Vector2 pref_vel_;
    double waypoint_offset_ = 0.0;

    //Counter clockwise polygon points centered around origin, facing x positive direction
    std::vector<RVO::Vector2> box_size_ = {RVO::Vector2(-0.25, 0.25), RVO::Vector2(-0.25, -0.25), RVO::Vector2(0.25, -0.25), RVO::Vector2(0.25, 0.25), RVO::Vector2(-0.25, 0.25), RVO::Vector2(-0.25, -0.25), RVO::Vector2(0.25, -0.25), RVO::Vector2(0.25, 0.25)};

    void updateWaypoint(double dist_)
    {
        if (dist_ < distance_threshold_)
        {
            current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
            cur_waypoint_offset_ = RVO::Vector2((float)std::rand() / RAND_MAX * waypoint_offset_ - waypoint_offset_ / 2,
                                                (float)std::rand() / RAND_MAX * waypoint_offset_ - waypoint_offset_ / 2);
        }
    }

    void decrementWaypoint()
    {
        if(current_waypoint_ == 0)
            current_waypoint_ = waypoints_.size() - 1;

        else 
            current_waypoint_--;
    }

    RVO::Vector2 getPrefVel()
    {
        RVO::Vector2 current_waypoint = waypoints_[current_waypoint_] + cur_waypoint_offset_;
        RVO::Vector2 dir1 = RVO::normalize(RVO::Vector2(current_waypoint.x() - odom_.pose.pose.position.x, current_waypoint.y() - odom_.pose.pose.position.y));

        double dist = sqrt(pow(current_waypoint.x() - odom_.pose.pose.position.x, 2) + pow(current_waypoint.y() - odom_.pose.pose.position.y, 2));
        pref_vel_ = pref_vel_ * (1 - waypoint_filter_const_) + dir1 * waypoint_filter_const_;

        //Check if this agent has been stuck at the same spot
        double dist_to_last_odom = sqrt(pow(last_odom_.pose.pose.position.x - odom_.pose.pose.position.x, 2) +
                                        pow(last_odom_.pose.pose.position.y - odom_.pose.pose.position.y, 2));

        //If distance is greater than update threshold, update last odom
        if(dist_to_last_odom > odom_update_thresh_)
        {
            last_odom_ = odom_;
            last_odom_.header.stamp = ros::Time::now();
        }

        //Else if distance is greater and it has been too long, decrement waypoint to reverse direction and set new odom
        else if((ros::Time::now() - last_odom_.header.stamp).toSec() > 3.0 && dist_to_last_odom < odom_update_thresh_)
        {
            ROS_INFO("Revert waypoint");
            decrementWaypoint();
            last_odom_ = odom_;
            last_odom_.header.stamp = ros::Time::now();
        }

        updateWaypoint(dist);

        return RVO::normalize(pref_vel_);
    }

    std::vector<RVO::Vector2> getBoundingBox(RVO::Vector2 &heading, RVO::Vector2 &center)
    {
        std::vector<RVO::Vector2> bounding_box;
        auto heading_rot = heading.rotate(90);
        bounding_box.push_back(center - 0.25 * heading + 0.25 * heading_rot);
        bounding_box.push_back(center + 0.25 * heading + 0.25 * heading_rot);
        bounding_box.push_back(center + 0.25 * heading - 0.25 * heading_rot);
        bounding_box.push_back(center - 0.25 * heading - 0.25 * heading_rot);

        //Rotate original box size by heading amount
        // double yaw = atan2(heading.y(), heading.x());
        // for(auto &corner : bounding_box)
        //     corner.rotate(yaw);

        // std::vector<RVO::Vector2> bounding_box = box_size_;
        // RVO::Vector2 pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        // //Offset entire box by current position
        // for(auto &corner : bounding_box)
        //     corner += pos;

        return bounding_box;
    }

    void normalizeAngle(double &heading)
    {
        while (heading > M_PI)
            heading -= 2 * M_PI;

        while (heading <= -M_PI)
            heading += 2 * M_PI;
    }

    double updateHeading(double new_heading)
    {
        normalizeAngle(new_heading);

        if (heading_ == RVO::RVO_ERROR)
            heading_ = new_heading;

        else
        {
            //Get difference between current heading and next heading to get incremenet
            double diff_heading = new_heading - heading_;
            if (diff_heading > M_PI)
                diff_heading -= 2 * M_PI;

            else if (diff_heading <= -M_PI)
                diff_heading += 2 * M_PI;

            double new_heading = heading_ + diff_heading;
            heading_ = heading_ * (1 - heading_filter_const_) + heading_filter_const_ * new_heading;

            //Maintain heading values in -M_PI exclusive to M_PI inclusive range
            normalizeAngle(heading_);
        }

        return heading_;
    }

private:
};

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

//GAMMA scenario parameters
std::string scenario_name = "doorway";
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