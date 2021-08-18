#include <gamma_simulator/gamma_simulator_node.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <ctime>

enum AGENT_TYPES
{
    ROBOT,
    PEDESTRIAN
};

void setSeed(int seed)
{
    if (seed != 0)
    {
        ROS_INFO_STREAM("Using fixed seed of " << seed);
        std::srand(seed);
    }

    else
        std::srand(std::time(nullptr));
}

bool isQuaternionValid(const geometry_msgs::Quaternion &quat)
{
    if (fabs(pow(quat.x, 2) + pow(quat.y, 2) + pow(quat.z, 2) + pow(quat.w, 2) - 1.0) > 0.001)
        return false;

    return true;
}

bool addPedAgent(RVO::RVOSimulator *sim, const RVO::Vector2 &pos, AgentInfo &agent)
{
    agent.id_ = gamma_sim_->addAgent(pos);
    if (agent.id_ == RVO::RVO_ERROR)
        return false;

    else
    {
        gamma_sim_->setAgentResDecRate(agent.id_, 7.0);
        gamma_sim_->setAgentAttentionRadius(agent.id_, 3.0, 1.0);
        gamma_sim_->setAgentBehaviorType(agent.id_, RVO::AgentBehaviorType::GammaWithoutPoly);
        agent.heading_filter_const_ = heading_filter_const;
        agent.waypoint_filter_const_ = waypoint_filter_const;
    }

    return true;
}

//Set-up simulation parameters
bool setupGAMMA()
{
    ROS_INFO("Setting up GAMMA");

    //Set random seed for slight pertubation in pref vel
    setSeed(seed);

    //Setup GAMMA sim basic parameters
    gamma_sim_ = new RVO::RVOSimulator();
    gamma_sim_->setTimeStep(timeStep);
    gamma_sim_->setAgentDefaults(neighborDist, (size_t)maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);

    //Need to set bounding corners to use_polygon, gamma_sim_->setAgentBoundingBoxCorners
    GammaParams::use_polygon = use_polygon;
    GammaParams::consider_kinematics = consider_kinematics;
    GammaParams::use_dynamic_resp = use_dynamic_resp;
    GammaParams::use_dynamic_att = use_dynamic_att;

    //Add all polygons and agent's current position and waypoints to gamma_sim
    for (const auto &obs : obstacles_)
    {
        auto res = gamma_sim_->addObstacle(obs);
        if (res == RVO::RVO_ERROR)
            ROS_INFO("Failed to add obstacle");
    }

    ROS_INFO_STREAM("Number of obstacle vertices in GAMMA: " << gamma_sim_->getNumObstacleVertices());
    gamma_sim_->processObstacles();
    ROS_INFO("Processed obstacles");

    for (auto &agent : agents_)
    {
        if (!addPedAgent(gamma_sim_, RVO::Vector2(agent.odom_.pose.pose.position.x, agent.odom_.pose.pose.position.y), agent))
            ROS_INFO("Failed to add agent");
    }

    ROS_INFO_STREAM("Number of agents in GAMMA: " << gamma_sim_->getNumAgents());
    ROS_INFO("Setup GAMMA complete");
    return true;
}

bool resetGAMMA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    std::lock_guard<std::mutex> guard(valid_mutex);
    gamma_sim_->clearAllAgents();
    robot_info_.id_ = -1;
    agents_ = agents_initial_;
    global_plan_waypoint.orientation.x = 0;
    global_plan_waypoint.orientation.y = 0;
    global_plan_waypoint.orientation.z = 0;
    global_plan_waypoint.orientation.w = 0;

    setSeed(seed);

    //Add back all agents
    for (auto &agent : agents_)
    {
        if (!addPedAgent(gamma_sim_, RVO::Vector2(agent.odom_.pose.pose.position.x, agent.odom_.pose.pose.position.y), agent))
            ROS_INFO("Failed to add agent after reset");
    }
    ROS_INFO("Reset complete");
    resp.success = true;
    return true;
}

//Read obstacles file to get all obstacle polygons
bool readObstacles(const std::string &file)
{
    std::ifstream obs_file;
    obs_file.open(file, std::ifstream::in);

    if (obs_file.fail() || !obs_file.is_open())
        return false;

    std::string colon_delim = ":";
    std::string comma_delim = ",";

    //Obstacles are polygons of varying vertices. Obstacle polygons in file are separated by a newline
    std::string line;
    while (std::getline(obs_file, line))
    {
        //Split string by colon, delimiter between obstacle vertices
        size_t pos = 0;
        std::string obstacle_string;
        std::vector<RVO::Vector2> new_obs;
        line += colon_delim; //Append delimiter to allow reading of last token
        while ((pos = line.find(colon_delim)) != std::string::npos)
        {
            obstacle_string = line.substr(0, pos);
            obstacle_string += comma_delim; //Append delimiter to allow reading of last token

            //Split string by comma, delimiter between points in a vertex
            size_t vertex_pos = 0;
            std::string vertex_string;
            std::vector<float> vertex;
            while ((vertex_pos = obstacle_string.find(comma_delim)) != std::string::npos)
            {
                vertex_string = obstacle_string.substr(0, vertex_pos);
                vertex.push_back(std::stof(vertex_string));

                obstacle_string.erase(0, vertex_pos + comma_delim.length());
            }

            new_obs.push_back(RVO::Vector2(vertex[0], vertex[1]));
            line.erase(0, pos + colon_delim.length());
        }

        obstacles_.push_back(new_obs);
    }

    return true;
}

bool publishObstaclesViz()
{
    visualization_msgs::MarkerArray all_obstacles;
    for (int i = 0; i < obstacles_.size(); ++i)
    {
        auto &obs = obstacles_[i];
        visualization_msgs::Marker obstacle;
        obstacle.header.frame_id = "odom";
        obstacle.ns = "obstacle";
        obstacle.id = i;
        obstacle.type = visualization_msgs::Marker::LINE_STRIP;
        obstacle.action = 0;
        obstacle.pose.orientation.w = 1.0;
        obstacle.scale.x = 0.1;
        obstacle.color.a = 1.0;
        obstacle.color.r = 1.0;
        obstacle.color.b = 0.8;

        for (const auto &vertex : obs)
        {
            geometry_msgs::Point point;
            point.x = vertex.x();
            point.y = vertex.y();
            obstacle.points.emplace_back(std::move(point));
        }

        //Duplicate first point to close the polygon loop
        obstacle.points.push_back(obstacle.points[0]);

        all_obstacles.markers.push_back(obstacle);
    }

    obstacles_viz_pub_.publish(all_obstacles);
}

//Read scenario file to get all agents and their respective waypoints
bool readAgents(const std::string &file)
{
    std::ifstream agent_file;
    agent_file.open(file, std::ifstream::in);

    if (agent_file.fail())
        return false;

    std::string line;
    std::string comma_delim = ",";
    while (std::getline(agent_file, line))
    {
        size_t pos = 0;
        std::string agent_param;
        int token_count = 0;
        std::vector<std::string> waypoints;
        line += comma_delim; //Append delimiter to allow reading of last token
        int num_agents;
        double x, y, dx, dy, trigger_x, trigger_y, trigger_radius;
        bool use_trigger = false;
        while ((pos = line.find(comma_delim)) != std::string::npos)
        {
            agent_param = line.substr(0, pos);
            switch (token_count)
            {
            case 0:
                num_agents = std::stoi(agent_param);
                break;

            case 1:
                x = std::stod(agent_param);
                break;

            case 2:
                y = std::stod(agent_param);
                break;

            case 3:
                dx = std::stod(agent_param);
                break;

            case 4:
                dy = std::stod(agent_param);
                break;

            case 5:
                trigger_x = std::stod(agent_param);
                break;

            case 6:
                trigger_y = std::stod(agent_param);
                break;

            case 7:
                trigger_radius = std::stod(agent_param);
                break;

            case 8:
                if (agent_param == "false")
                    use_trigger = false;

                else if (agent_param == "true")
                    use_trigger = true;
                break;

            //Remaining parameters are waypoints
            default:
                waypoints.push_back(agent_param);
                break;
            }

            line.erase(0, pos + comma_delim.length());
            token_count++;
        }

        //Once line is processed, create the agents and add them into gamma

        for (int i = 0; i < num_agents; ++i)
        {
            AgentInfo temp_agent;
            temp_agent.use_trigger_point = use_trigger;
            temp_agent.trigger_point = RVO::Vector2(trigger_x, trigger_y);
            temp_agent.trigger_point_radius = trigger_radius;
            for (const auto &wp : waypoints)
                temp_agent.waypoints_.push_back(waypoints_[wp]);
            RVO::Vector2 pertubation((float)std::rand() / RAND_MAX * dx - dx / 2.0,
                                     (float)std::rand() / RAND_MAX * dy - dy / 2.0);
            temp_agent.odom_.pose.pose.position.x = x + pertubation.x();
            temp_agent.odom_.pose.pose.position.y = y + pertubation.y();
            agents_.push_back(temp_agent);
        }
    }

    agents_initial_ = agents_;
}

bool readWaypoints(const std::string &file)
{
    ROS_INFO("Read waypoints");
    std::ifstream waypoints_file;
    waypoints_file.open(file, std::ifstream::in);

    if (waypoints_file.fail())
    {
        ROS_ERROR("Failed to read waypoints");
        return false;
    }

    std::string waypoint_name;
    float x, y;
    while (waypoints_file >> waypoint_name >> x >> y)
        waypoints_[waypoint_name] = RVO::Vector2(x, y);
}

//Publish position to ROS
bool publishPedestrianPosition()
{
    gamma_simulator::AgentStates agent_states;
    for (int i = 0; i < agents_.size(); ++i)
    {
        //Get agent position from gamma
        auto agent_pos = gamma_sim_->getAgentPosition(agents_[i].id_);
        nav_msgs::Odometry state;
        state.header.seq = i;
        state.pose.pose.position.x = agent_pos.x();
        state.pose.pose.position.y = agent_pos.y();

        //Filter agent heading from gamma
        auto agent_heading = gamma_sim_->getAgentHeading(agents_[i].id_);
        double yaw = atan2(agent_heading.y(), agent_heading.x()) + M_PI; //Why do I need to add PI? Because gazebo heading orientation is different?
        yaw = agents_[i].updateHeading(yaw);

        state.pose.pose.orientation.z = cos(yaw / 2.0);
        state.pose.pose.orientation.w = -sin(yaw / 2.0);

        agent_states.agent_states.emplace_back(state);
    }

    agent_states_pub_.publish(agent_states);

    return true;
}

bool publishWaypointMarkers()
{
    visualization_msgs::MarkerArray waypoint_markers;
    int i = 0;
    for(const auto &pair : waypoints_)
    {
        RVO::Vector2 cur_waypoint = pair.second;

        //Add waypoint flat circle marker
        visualization_msgs::Marker wp_marker;
        wp_marker.header.frame_id = "map";
        wp_marker.ns = std::to_string(i);
        wp_marker.id = i;
        wp_marker.action = 0;
        wp_marker.type = 3;
        wp_marker.pose.position.x = cur_waypoint.x();
        wp_marker.pose.position.y = cur_waypoint.y();
        wp_marker.pose.position.z = 0;
        wp_marker.scale.x = 0.9;
        wp_marker.scale.y = 0.9;
        wp_marker.scale.z = 0.01;
        wp_marker.color.r = 0;
        wp_marker.color.g = 1;
        wp_marker.color.b = 1;
        wp_marker.color.a = 1;
        wp_marker.lifetime = ros::Duration(0);

        //Add label text for waypoint
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.ns = std::to_string(i) + std::string(" Text");
        text_marker.id = i + waypoints_.size();
        text_marker.action = 0;
        text_marker.type = 9;
        text_marker.pose.position.x = cur_waypoint.x();
        text_marker.pose.position.y = cur_waypoint.y();
        text_marker.pose.position.z = 0.05;
        text_marker.scale.z = 0.9;
        text_marker.color.r = 0;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.text = std::string(pair.first);
        text_marker.lifetime = ros::Duration(0);

        waypoint_markers.markers.emplace_back(std::move(wp_marker));
        waypoint_markers.markers.emplace_back(std::move(text_marker));
        ++i;
    }

    all_waypoints_pub_.publish(waypoint_markers);
    return true;
}

bool publishSimRobotVelocity()
{
    robot_vel_pub_.publish(robot_twist_cmd);
    return true;
}

bool readParams(ros::NodeHandle &nh)
{
    if (!nh.getParam("neighborDist", neighborDist))
        ROS_WARN_STREAM("Parameter neighborDist not set. Using default setting: " << neighborDist);
    if (!nh.getParam("maxNeighbors", maxNeighbors))
        ROS_WARN_STREAM("Parameter maxNeighbors not set. Using default setting: " << maxNeighbors);
    if (!nh.getParam("timeHorizon", timeHorizon))
        ROS_WARN_STREAM("Parameter timeHorizon not set. Using default setting: " << timeHorizon);
    if (!nh.getParam("timeHorizonObst", timeHorizonObst))
        ROS_WARN_STREAM("Parameter timeHorizonObst not set. Using default setting: " << timeHorizonObst);
    if (!nh.getParam("radius", radius))
        ROS_WARN_STREAM("Parameter radius not set. Using default setting: " << radius);
    if (!nh.getParam("maxSpeed", maxSpeed))
        ROS_WARN_STREAM("Parameter maxSpeed not set. Using default setting: " << maxSpeed);
    if (!nh.getParam("rate", rate))
        ROS_WARN_STREAM("Parameter rate not set. Using default setting: " << rate);
    if (!nh.getParam("heading_filter_time_const", heading_filter_time_const))
        ROS_WARN_STREAM("Parameter heading_filter_time_const not set. Using default setting: " << heading_filter_time_const);
    if (!nh.getParam("waypoint_filter_time_const", waypoint_filter_time_const))
        ROS_WARN_STREAM("Parameter waypoint_filter_time_const not set. Using default setting: " << waypoint_filter_time_const);
    if (!nh.getParam("simulate_robot", simulate_robot))
        ROS_WARN_STREAM("Parameter simulate_robot not set. Using default setting: " << simulate_robot);
    if (!nh.getParam("seed", seed))
        ROS_WARN_STREAM("Parameter seed not set. Using default setting: " << seed);
    if (!nh.getParam("scenario_name", scenario_name))
        ROS_WARN_STREAM("Parameter scenario_name not set. Using default setting: " << scenario_name);
    if (!nh.getParam("robot_radius", robot_radius))
        ROS_WARN_STREAM("Parameter robot_radius not set. Using default setting: " << robot_radius);

    //PID Parameters
    nh.getParam("steering_p", steering_p);
    nh.getParam("steering_i", steering_i);
    nh.getParam("steering_d", steering_d);
    nh.getParam("steering_f", steering_f);
    nh.getParam("steering_output_ramp", steering_output_ramp);
    nh.getParam("steering_output_desc", steering_output_desc);
    nh.getParam("steering_max_output", steering_max_output);
    nh.getParam("steering_min_output", steering_min_output);
    nh.getParam("steering_max_i_output", steering_max_i_output);
    nh.getParam("velocity_p", velocity_p);
    nh.getParam("velocity_i", velocity_i);
    nh.getParam("velocity_d", velocity_d);
    nh.getParam("velocity_f", velocity_f);
    nh.getParam("velocity_output_ramp", velocity_output_ramp);
    nh.getParam("velocity_output_desc", velocity_output_desc);
    nh.getParam("velocity_max_output", velocity_max_output);
    nh.getParam("velocity_min_output", velocity_min_output);
    nh.getParam("velocity_max_i_output", velocity_max_i_output);
    nh.getParam("pid_freq", pid_freq);
}

bool setupPID()
{
    steering_pid.setPIDF(steering_p, steering_i, steering_d, steering_f);
    steering_pid.setMaxIOutput(steering_max_i_output);
    steering_pid.setOutputLimits(steering_min_output, steering_max_output);
    steering_pid.setOutputRampRate(steering_output_ramp);
    steering_pid.setOutputDescentRate(steering_output_desc);
    steering_pid.setFrequency(pid_freq);

    velocity_pid.setPIDF(velocity_p, velocity_i, velocity_d, velocity_f);
    velocity_pid.setMaxIOutput(velocity_max_i_output);
    velocity_pid.setOutputLimits(velocity_min_output, velocity_max_output);
    velocity_pid.setOutputRampRate(velocity_output_ramp);
    velocity_pid.setOutputDescentRate(velocity_output_desc);
    velocity_pid.setFrequency(pid_freq);
}

//Global plan should constantly replan to ensure that start position is on robot
void globalPlanCB(const nav_msgs::PathConstPtr &msg)
{
    if (msg->poses.size() == 0)
        return;

    //Get waypoint from global plan
    double dist = 0;
    auto new_waypoint = msg->poses.back().pose;
    for (int i = 1; i < msg->poses.size(); ++i)
    {
        double temp_dist = sqrt(pow(msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x, 2) +
                                pow(msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y, 2));

        dist += temp_dist;
        if (dist >= waypoint_distance)
        {
            new_waypoint = msg->poses[i].pose;
            break;
        }
    }

    //Transform to odom frame
    geometry_msgs::TransformStamped odom_to_map_tf;
    try
    {
        odom_to_map_tf = tf_buf.lookupTransform("odom", msg->header.frame_id, ros::Time(0), ros::Duration(1 / rate));
        tf2::doTransform(new_waypoint, new_waypoint, odom_to_map_tf);
        global_plan_waypoint = new_waypoint;

        //Goal reached
        if (dist < 0.2)
            global_plan_waypoint = geometry_msgs::Pose();

        geometry_msgs::PoseStamped pub_pose;
        pub_pose.pose = global_plan_waypoint;
        pub_pose.header.frame_id = "odom";
        waypoint_pub_.publish(pub_pose);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void robotControllerTimer()
{
    //robot_info_.prev_vel_ is current velocity. TODO: Do we need mutex for robot_info_?
    static ros::Time last = ros::Time::now();
    while (ros::ok())
    {
        std::lock_guard<std::mutex> guard(valid_mutex);
        //If time is less than control frequency, skip cycle
        if ((ros::Time::now() - last).toSec() < 1.0 / pid_freq)
            continue;

        //GAMMA not ready
        if (robot_info_.id_ == -1)
            continue;

        auto pref_vel = gamma_sim_->getAgentPrefVelocity(robot_info_.id_);
        if (pref_vel.x() == 0.0 && pref_vel.y() == 0.0)
        {
            robot_twist_cmd.linear.x = 0.0;
            robot_twist_cmd.angular.z = 0.0;
            continue;
        }

        //Get the robot's command velocity from gamma
        RVO::Vector2 robot_cmd_vel = gamma_sim_->getAgentVelocity(robot_info_.id_);

        //Get heading difference between robot command velocity and current velocity, diff = cmd - cur
        double heading_diff = atan2(robot_cmd_vel.y(), robot_cmd_vel.x()) - robot_info_.heading_;
        if (heading_diff > M_PI)
            heading_diff = -2 * M_PI + heading_diff;

        else if (heading_diff < -M_PI)
            heading_diff = 2 * M_PI + heading_diff;

        //Get speed difference
        double speed_diff = RVO::abs(robot_cmd_vel) - RVO::abs(robot_info_.pref_vel_);

        robot_twist_cmd.angular.z = std::max(std::min(heading_diff, 1.0), -1.0);
        robot_twist_cmd.linear.x = RVO::abs(robot_cmd_vel);

        //Cap total magnitude of angular and linear velocity to simulate joystick behavior
        if (fabs(robot_twist_cmd.linear.x) + fabs(robot_twist_cmd.angular.z) > 1.0)
            robot_twist_cmd.linear.x = robot_twist_cmd.linear.x / fabs(robot_twist_cmd.linear.x) * (1.0 - fabs(robot_twist_cmd.angular.z));

        // steering_pid.getOutput()
        last = ros::Time::now();
    }
}

bool setRobotVelocityConvex()
{
    double max_angle = 180.0, angle_res = 5;
    double max_speed = 1.0, speed_res = 0.05;
    int speed_size = max_speed / speed_res;
    int angle_size = max_angle / angle_res;
    double error_bound = 0.5;
    RVO::Vector2 y_vel(0, 1.0);

    std::vector<RVO::Vector2> boundary_v;
    boundary_v.resize(2 * angle_size);
    boundary_v[0] = RVO::Vector2(0, 0);
    //Find the trackable velocities for all angles around the robot
    for (int i = 0; i < angle_size; ++i)
    {
        double cur_speed = (max_angle - i * angle_res) / max_angle * max_speed;
        if (cur_speed < 0)
            cur_speed = 0;
        //Straight ahead velocity
        if (i == 0)
        {
            boundary_v[1] = (cur_speed)*y_vel;
        }
        //Left and right velocity
        else
        {
            boundary_v[2 * i] = (cur_speed)*y_vel.rotate(i * angle_res);
            boundary_v[2 * i + 1] = (cur_speed)*y_vel.rotate(-i * angle_res);
        }
    }

    auto temp = RVO::makeConvexHull(boundary_v);
    std::reverse(temp.begin(), temp.end());
    gamma_sim_->setAgentVelocityConvex(robot_info_.id_, temp);

    return true;
}

void robotOdom()
{
    if (gamma_sim_ == nullptr)
        return;

    static ros::Time last = ros::Time::now();
    double dt = (ros::Time::now() - last).toSec();
    //Get current robot position for tracking velocity
    geometry_msgs::TransformStamped odom_to_map_tf;
    try
    {
        odom_to_map_tf = tf_buf.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1 / rate));

        geometry_msgs::Pose temp_pose;
        temp_pose.orientation.w = 1;
        tf2::doTransform(temp_pose, temp_pose, odom_to_map_tf);

        //Calculate velocity of robot
        double vx = (temp_pose.position.x - robot_info_.odom_.pose.pose.position.x) / dt;
        double vy = (temp_pose.position.y - robot_info_.odom_.pose.pose.position.y) / dt;
        robot_info_.pref_vel_ = RVO::Vector2(vx, vy);

        //Get actual heading of robot from tf
        robot_info_.heading_ = 2 * atan2(-temp_pose.orientation.w, temp_pose.orientation.z) + M_PI;
        if (robot_info_.heading_ > M_PI)
            robot_info_.heading_ = -2 * M_PI + robot_info_.heading_;
        else if (robot_info_.heading_ < -M_PI)
            robot_info_.heading_ = 2 * M_PI - robot_info_.heading_;

        //Update robot's last position
        robot_info_.odom_.pose.pose = temp_pose;

        //Robot has not been added to GAMMA yet
        if (robot_info_.id_ == -1)
        {
            robot_info_.id_ = gamma_sim_->addAgent(RVO::Vector2(robot_info_.odom_.pose.pose.position.x, robot_info_.odom_.pose.pose.position.y));
            if (robot_info_.id_ == RVO::RVO_ERROR)
                ROS_ERROR("Failed to add robot agent");

            else
            {
                gamma_sim_->setAgentResDecRate(robot_info_.id_, 0.10);
                gamma_sim_->setAgentAttentionRadius(robot_info_.id_, 2.0, 0.4);
                gamma_sim_->setAgentTag(robot_info_.id_, "Car");
                gamma_sim_->setAgentBehaviorType(robot_info_.id_, RVO::AgentBehaviorType::GammaWithoutPoly);
                gamma_sim_->setAgentRadius(robot_info_.id_, robot_radius);
                setRobotVelocityConvex();
                ROS_INFO("Robot agent added to GAMMA");
            }
        }

        gamma_sim_->setAgentPosition(robot_info_.id_, RVO::Vector2(robot_info_.odom_.pose.pose.position.x, robot_info_.odom_.pose.pose.position.y));

        //Set robot's position and velocity
        if (!simulate_robot)
        {
            gamma_sim_->setAgentVelocity(robot_info_.id_, robot_info_.pref_vel_);
            gamma_sim_->setAgentPrefVelocity(robot_info_.id_, robot_info_.pref_vel_);
        }

        //Robot is being simulated, set preferred velocity using waypoint
        else
        {
            //Set agent heading because this is set internally based on velocity, which is inaccurate
            gamma_sim_->setAgentHeading(robot_info_.id_, RVO::Vector2(cos(robot_info_.heading_), sin(robot_info_.heading_)));

            if (isQuaternionValid(global_plan_waypoint.orientation))
            {
                RVO::Vector2 dir = RVO::normalize(RVO::Vector2(global_plan_waypoint.position.x - robot_info_.odom_.pose.pose.position.x,
                                                               global_plan_waypoint.position.y - robot_info_.odom_.pose.pose.position.y));
                gamma_sim_->setAgentPrefVelocity(robot_info_.id_, dir);
            }

            else
            {
                ROS_INFO("Global waypoint is invalid");
                gamma_sim_->setAgentPrefVelocity(robot_info_.id_, RVO::Vector2(0.0, 0.0));
            }
        }

        last = ros::Time::now();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

//Run simulation step by step
bool runStep()
{
    //Get robot position and heading
    robotOdom();

    //Get preferred velocity based on current waypoint
    RVO::Vector2 robot_pos(robot_info_.odom_.pose.pose.position.x, robot_info_.odom_.pose.pose.position.y);
    for(int i = 0; i < agents_.size(); ++i)
    {
        auto &agent = agents_[i];
        if (agent.use_trigger_point)
        {
            //Move agent off frame when trajectory complete
            if (agent.final_waypoint_reached)
            {
                agent.pref_vel_ = RVO::Vector2(0, 0);
                agent.odom_.pose.pose.position.x = -1000;
                agent.odom_.pose.pose.position.y = -1000;
                gamma_sim_->setAgentPosition(agent.id_, RVO::Vector2(agent.odom_.pose.pose.position.x, agent.odom_.pose.pose.position.y));
            }

            //Send robot position to agent for checking
            else
                agent.checkTriggerPoint(robot_pos);
        }

        RVO::Vector2 pref_vel = agent.getPrefVel();
        gamma_sim_->setAgentPrefVelocity(agent.id_, pref_vel);
    }

    gamma_sim_->doStep();

    //Update positions in local store
    for (auto &agent : agents_)
    {
        RVO::Vector2 pos = gamma_sim_->getAgentPosition(agent.id_);
        agent.odom_.pose.pose.position.x = pos.x();
        agent.odom_.pose.pose.position.y = pos.y();
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamma_simulator");
    ros::NodeHandle nh("~");

    //Start tf buffer
    tf2_ros::TransformListener listener(tf_buf);

    //Read ros params
    readParams(nh);

    //Find scenario files from gamma_simulator folder after read params
    std::string package_path = ros::package::getPath("gamma_simulator");
    waypoints_file = package_path + "/scenario/" + scenario_name + "/waypoints.txt";
    agents_file = package_path + "/scenario/" + scenario_name + "/agents.txt";
    obstacles_file = package_path + "/scenario/" + scenario_name + "/obstacles.txt";

    agent_states_pub_ = nh.advertise<gamma_simulator::AgentStates>("/gamma_simulator/agent_states", 1);
    obstacles_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/gamma_simulator/obstacles_viz", 1, true);
    all_waypoints_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/gamma_simualtor/all_waypoints", 1, true);

    if (simulate_robot)
    {
        robot_sim_thread_ = std::thread(&robotControllerTimer);
        robot_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gamma_simulator/waypoint", 1);
        global_plan_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 1, &globalPlanCB);
    }

    // robot_odom_timer_ = nh.createTimer(ros::Duration(1.0 / rate), &robotOdomTimer);

    //Calculate time related parameters
    timeStep = 1 / rate;
    heading_filter_const = 1 - exp(-timeStep / heading_filter_time_const); //Fix heading time constant, depending on ros rate
    waypoint_filter_const = 1 - exp(-timeStep / waypoint_filter_time_const);

    readWaypoints(waypoints_file);
    readObstacles(obstacles_file);
    readAgents(agents_file);
    publishWaypointMarkers();
    setupPID();
    setupGAMMA();
    publishObstaclesViz();

    //Advertise service only after gamma is setup to indicate simulator is ready
    reset_gamma_serv_ = nh.advertiseService("/gamma_simulator/reset_agents", &resetGAMMA);

    ros::Rate r(rate);
    ROS_INFO("Start GAMMA");
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        runStep();
        publishPedestrianPosition();

        if (simulate_robot)
            publishSimRobotVelocity();
    }

    if (simulate_robot)
    {
        std::cout << "\n\nWaiting for robot simulation thread to end\n";
        robot_sim_thread_.join();
        std::cout << "Simulation thread exited\n";
    }

    return 0;
}
