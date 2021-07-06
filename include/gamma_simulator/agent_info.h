#ifndef AGENT_INFO_H_
#define AGENT_INFO_H_

#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gamma_simulator/RVO.h>
#include <ros/ros.h>

class AgentInfo
{
public:
	/**
     * Vector of waypoints for this agent loaded from scenario files
     **/
	std::vector<RVO::Vector2> waypoints_;

	/**
     * Index of current goal waypoint, with reference to waypoints_ vector
     **/
	int current_waypoint_ = 0; //Start with 1 because prefVel will decrement this on initialization

	/**
     * ID of agent in GAMMA
     **/
	int id_ = -1;

	/**
     * Time threshold in seconds for if the agent is in the same position for too long, agent is considered stuck
     **/
	double stuck_time_threshold = 3;

	/**
     * Current position of agent
     **/
	nav_msgs::Odometry odom_;

	/**
     * Last position of agent
     **/
	nav_msgs::Odometry last_odom_;

	/**
     * Waypoint distance threshold. Agent is considered reached goal if agent is within this threshold to waypoint
     **/
	double distance_threshold_ = 0.3;

	/**
     * If this threshold exceeded, "last position" of agent is updated. Last position is used to check if robot is stuck for extended period of time
     **/
	double odom_update_thresh_ = 0.2;

	/**
     * Current agent heading
     **/
	double heading_ = RVO::RVO_ERROR;

	/**
     * Exponential filter constant for smoothing out changes in agent heading
     **/
	double heading_filter_const_ = 0.90;

	/**
     * Exponential filter constant for smoothing out changes in waypoint direction
     **/
	double waypoint_filter_const_ = 0.90;

	/**
     * Current offset from current goal
     **/
	RVO::Vector2 cur_waypoint_offset_ = RVO::Vector2(0, 0);

	/**
     * Preferred velocity given robot location and waypoint
     **/
	RVO::Vector2 pref_vel_;

	/**
     * Max random offset radius in meters around a waypoint for the pedestrian
     **/
	double waypoint_offset_ = 0.0;

	/**
	 * Flag to indicate if the final waypoint was reached. Used when trigger point is used
	 **/
	bool final_waypoint_reached = false;

	//Variables to trigger pedestrian motion only when robot is within a certain distance from trigger point
	bool point_triggered = false;
	RVO::Vector2 trigger_point;
	double trigger_point_radius = 1;
	bool use_trigger_point = false;

	void updateWaypoint(double dist_)
	{
		if (dist_ < distance_threshold_)
		{
			current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
			cur_waypoint_offset_ = RVO::Vector2((float)std::rand() / RAND_MAX * waypoint_offset_ - waypoint_offset_ / 2,
							    (float)std::rand() / RAND_MAX * waypoint_offset_ - waypoint_offset_ / 2);
			if(current_waypoint_ == 0)
				final_waypoint_reached = true;
		}
	}

	void decrementWaypoint()
	{
		if (current_waypoint_ == 0)
			current_waypoint_ = waypoints_.size() - 1;

		else
			current_waypoint_--;
	}

	bool checkTriggerPoint(const RVO::Vector2 &robot_position)
	{
		double dist = sqrt(pow(robot_position.x() - trigger_point.x(), 2) + pow(robot_position.y() - trigger_point.y(), 2));
		if (dist <= trigger_point_radius)
			point_triggered = true;
	}

	RVO::Vector2 getPrefVel()
	{
		//Trigger point is being used and triggered, or not using trigger point at all
		if ((use_trigger_point && point_triggered) || !use_trigger_point)
		{
			RVO::Vector2 current_waypoint = waypoints_[current_waypoint_] + cur_waypoint_offset_;
			RVO::Vector2 dir1 = RVO::normalize(RVO::Vector2(current_waypoint.x() - odom_.pose.pose.position.x, current_waypoint.y() - odom_.pose.pose.position.y));

			double dist = sqrt(pow(current_waypoint.x() - odom_.pose.pose.position.x, 2) + pow(current_waypoint.y() - odom_.pose.pose.position.y, 2));
			pref_vel_ = pref_vel_ * (1 - waypoint_filter_const_) + dir1 * waypoint_filter_const_;
			updateWaypoint(dist);
		}

		//Using trigger point but not triggered yet
		else if (use_trigger_point && !point_triggered)
			pref_vel_ = RVO::Vector2(0, 0);

		//Check if this agent has been stuck at the same spot
		double dist_to_last_odom = sqrt(pow(last_odom_.pose.pose.position.x - odom_.pose.pose.position.x, 2) +
						pow(last_odom_.pose.pose.position.y - odom_.pose.pose.position.y, 2));

		//If distance is greater than update threshold, update last odom. If using trigger point but not triggered yet, continuously update to prevent
		//unneccesary reverting of current waypoint
		if (dist_to_last_odom > odom_update_thresh_ || (use_trigger_point && !point_triggered))
		{
			last_odom_ = odom_;
			last_odom_.header.stamp = ros::Time::now();
		}

		//Else if distance is less and it has been too long, decrement waypoint to reverse direction and set new odom
		else if ((ros::Time::now() - last_odom_.header.stamp).toSec() > stuck_time_threshold && dist_to_last_odom < odom_update_thresh_)
		{
			ROS_INFO("Revert waypoint");
			decrementWaypoint();
			last_odom_ = odom_;
			last_odom_.header.stamp = ros::Time::now();
		}

		return RVO::normalize(pref_vel_);
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

#endif