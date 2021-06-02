#ifndef GAMMA_SIMULATOR_NODE_H_
#define GAMMA_SIMULATOR_NODE_H_

#include <vector>
#include <string>
#include <gamma_simulator/Agent.h>
#include <gamma_simulator/GammaParams.h>
#include <gamma_simulator/RVO.h>
#include <gamma_simulator/RVOSimulator.h>
#include <gamma_simulator/AgentStates.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

class AgentInfo
{
public:
    std::vector<geometry_msgs::Point> waypoints_;
    int current_waypoint_ = 0;
    nav_msgs::Odometry odom_;
    double distance_threshold_ = 0.1;

    void updateWaypoint()
    {
        double dist = sqrt(pow(waypoints_[current_waypoint_].x - odom_.pose.pose.position.x, 2) +
                                   pow(waypoints_[current_waypoint_].y - odom_.pose.pose.position.y, 2));
        if (dist < distance_threshold_)
            current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
    }

private:
};

ros::Publisher robot_vel_pub_;
ros::Publisher agent_states_pub_;

RVO::RVOSimulator *gamma_sim_;
float timeStep = 0.1;
float neighborDist = 2.0;
size_t maxNeighbors = 5;
float timeHorizon = 2.0;
float timeHorizonObst = 2.0;
float radius = 0.35;
float maxSpeed = 1.0;
float rate = 50;

std::vector<std::vector<RVO::Vector2>> obstacles_;
std::vector<std::pair<size_t, AgentInfo>> agents_;
int robot_agent_id_;
gamma_simulator::AgentStates agent_states_;

#endif