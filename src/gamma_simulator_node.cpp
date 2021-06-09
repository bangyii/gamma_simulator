#include <gamma_simulator/gamma_simulator_node.h>
#include <fstream>
#include <iostream>
#include <ctime>

enum AGENT_TYPES
{
    ROBOT,
    PEDESTRIAN
};

//Set-up simulation parameters
bool setupGAMMA()
{
    ROS_INFO("Setting up GAMMA");

    //Set random seed for slight pertubation in pref vel
    std::srand(std::time(nullptr));

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
        agent.id_ = gamma_sim_->addAgent(RVO::Vector2(agent.odom_.pose.pose.position.x, agent.odom_.pose.pose.position.y));
        if (agent.id_ == RVO::RVO_ERROR)
            ROS_INFO("Failed to add agent");

        else
        {
            gamma_sim_->setAgentResDecRate(agent.id_, 10.0);
            gamma_sim_->setAgentAttentionRadius(agent.id_, 3.0, 1.0);
            agent.heading_filter_const_ = heading_filter_const;
            agent.waypoint_filter_const_ = waypoint_filter_const;
        }
    }

    ROS_INFO_STREAM("Number of agents in GAMMA: " << gamma_sim_->getNumAgents());
    ROS_INFO("Setup GAMMA complete");
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
        double x, y, dx, dy;
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
            for (const auto &wp : waypoints)
                temp_agent.waypoints_.push_back(waypoints_[wp]);
            RVO::Vector2 pertubation((float)std::rand() / RAND_MAX * dx - dx / 2.0,
                                     (float)std::rand() / RAND_MAX * dy - dy / 2.0);
            temp_agent.odom_.pose.pose.position.x = x + pertubation.x();
            temp_agent.odom_.pose.pose.position.y = y + pertubation.y();
            agents_.push_back(temp_agent);
        }
    }
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

//Run simulation step by step
bool runStep()
{
    //Get preferred velocity based on current waypoint
    for (auto &agent : agents_)
    {
        RVO::Vector2 pref_vel = agent.getPrefVel();

        gamma_sim_->setAgentPrefVelocity(agent.id_, pref_vel);
    }

    //Set velocity convex of robot based on current velocity

    gamma_sim_->doStep();

    //Update positions in local store
    for (auto &agent : agents_)
    {
        RVO::Vector2 pos = gamma_sim_->getAgentPosition(agent.id_);
        agent.odom_.pose.pose.position.x = pos.x();
        agent.odom_.pose.pose.position.y = pos.y();
    }

        RVO::Vector2 pos = gamma_sim_->getAgentVelocity(robot_info_.id_);
        ROS_INFO_STREAM(pos.x() << ", " << pos.y());

    return true;
}

bool publishRobotVelocity()
{
    auto robot_vel = gamma_sim_->getAgentVelocity(robot_agent_id_);
    auto robot_heading = gamma_sim_->getAgentHeading(robot_agent_id_);

    geometry_msgs::Twist joy_vel;
    joy_vel.linear.x = robot_vel.x();
    joy_vel.angular.z = robot_vel.y();

    return true;
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
        double yaw = atan2(agent_heading.y(), agent_heading.x()) + M_PI;
        yaw = agents_[i].updateHeading(yaw);

        RVO::Vector2 new_heading(cos(yaw / 2.0), -sin(yaw / 2.0));
        state.pose.pose.orientation.z = new_heading.x();
        state.pose.pose.orientation.w = new_heading.y();

        agent_states.agent_states.emplace_back(state);
    }

    agent_states_pub_.publish(agent_states);

    return true;
}

bool readParams(ros::NodeHandle &nh)
{
    if (!nh.getParam("timeStep", timeStep))
        ROS_WARN_STREAM("Parameter timeStep not set. Using default setting: " << timeStep);
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
    if (!nh.getParam("use_polygon", use_polygon))
        ROS_WARN_STREAM("Parameter use_polygon not set. Using default setting: " << use_polygon);
    if (!nh.getParam("consider_kinematics", consider_kinematics))
        ROS_WARN_STREAM("Parameter consider_kinematics not set. Using default setting: " << consider_kinematics);
    if (!nh.getParam("use_dynamic_resp", use_dynamic_resp))
        ROS_WARN_STREAM("Parameter use_dynamic_resp not set. Using default setting: " << use_dynamic_resp);
    if (!nh.getParam("use_dynamic_att", use_dynamic_att))
        ROS_WARN_STREAM("Parameter use_dynamic_att not set. Using default setting: " << use_dynamic_att);
    if (!nh.getParam("heading_filter_time_const", heading_filter_time_const))
        ROS_WARN_STREAM("Parameter heading_filter_time_const not set. Using default setting: " << heading_filter_time_const);
    if (!nh.getParam("waypoint_filter_time_const", waypoint_filter_time_const))
        ROS_WARN_STREAM("Parameter waypoint_filter_time_const not set. Using default setting: " << waypoint_filter_time_const);
    if (!nh.getParam("simulate_robot", simulate_robot))
        ROS_WARN_STREAM("Parameter simulate_robot not set. Using default setting: " << simulate_robot);
}

void robotOdomTimer(const ros::TimerEvent &e)
{
    if(gamma_sim_ == nullptr)
        return;

    double dt = (e.current_real - e.last_real).toSec();
    //Get current robot position for tracking velocity
    geometry_msgs::TransformStamped odom_to_map_tf;
    try
    {
        odom_to_map_tf = tf_buf.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1 / 20));

        geometry_msgs::Pose temp_pose;
        temp_pose.orientation.w = 1;
        tf2::doTransform(temp_pose, temp_pose, odom_to_map_tf);

        //Calculate velocity of robot
        double vx = (temp_pose.position.x - robot_info_.odom_.pose.pose.position.x)/dt;
        double vy = (temp_pose.position.y - robot_info_.odom_.pose.pose.position.y)/dt;
        robot_info_.pref_vel_ = RVO::Vector2(vx, vy);

        //Update robot's last position
        robot_info_.odom_.pose.pose = temp_pose;

        //Robot has not been added to GAMMA yet
        if(robot_info_.id_ == -1)
        {
            robot_info_.id_ = gamma_sim_->addAgent(RVO::Vector2(robot_info_.odom_.pose.pose.position.x, robot_info_.odom_.pose.pose.position.y));
            if(robot_info_.id_ == RVO::RVO_ERROR)
                ROS_ERROR("Failed to add robot agent");

            else
            {
                gamma_sim_->setAgentResDecRate(robot_info_.id_, 0.00);
                gamma_sim_->setAgentAttentionRadius(robot_info_.id_, 2.0, 0.4);
                gamma_sim_->setAgentTag(robot_info_.id_, "Car");
            }
        }

        //Set robot's position in gamma
        gamma_sim_->setAgentPosition(robot_info_.id_, RVO::Vector2(robot_info_.odom_.pose.pose.position.x, robot_info_.odom_.pose.pose.position.y));
        gamma_sim_->setAgentVelocity(robot_info_.id_, robot_info_.pref_vel_);
        gamma_sim_->setAgentPrefVelocity(robot_info_.id_, robot_info_.pref_vel_);
        gamma_sim_->setAgentRadius(robot_info_.id_, 0.3);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamma_simulator");
    ros::NodeHandle nh;

    //Start tf buffer
    tf2_ros::TransformListener listener(tf_buf);

    //Read ros params
    readParams(nh);

    robot_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    agent_states_pub_ = nh.advertise<gamma_simulator::AgentStates>("/gamma_simulator/agent_states", 1);
    obstacles_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/gamma_simulator/obstacles_viz", 1, true);

    if(!simulate_robot)
        robot_odom_timer_ = nh.createTimer(ros::Duration(1.0 / rate), &robotOdomTimer);

    //Calculate time related parameters
    timeStep = 1 / rate;
    heading_filter_const = 1 - exp(-timeStep / heading_filter_time_const); //Fix heading time constant, depending on ros rate
    waypoint_filter_const = 1 - exp(-timeStep / waypoint_filter_time_const);

    readWaypoints("/home/by/Desktop/waypoints.txt");
    readObstacles("/home/by/Desktop/obstacles.txt");
    readAgents("/home/by/Desktop/agents.txt");
    setupGAMMA();
    publishObstaclesViz();

    ros::Rate r(rate);
    ROS_INFO("Start GAMMA");
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        runStep();
        publishPedestrianPosition();
    }

    return 0;
}