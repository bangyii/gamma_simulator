#include <gamma_simulator/gamma_simulator_node.h>
#include <fstream>

enum AGENT_TYPES
{
    ROBOT,
    PEDESTRIAN
};

//Set-up simulation parameters
bool setupGAMMA()
{
    gamma_sim_ = new RVO::RVOSimulator(timeStep, neighborDist, maxNeighbors,
                                       timeHorizon, timeHorizonObst, radius,
                                       maxSpeed);

    //Add all polygons and agent's current position and waypoints to gamma_sim
    for (const auto &obs : obstacles_)
    {
        auto res = gamma_sim_->addObstacle(obs);
        if(res == RVO::RVO_ERROR)
            ROS_INFO("Failed to add obstacle");

        // ROS_INFO_STREAM("Number of obstacle vertices " << gamma_sim_->getObstacleVertex(res));
        // res = gamma_sim_->getNextObstacleVertexNo(res);
        // ROS_INFO_STREAM("Number of obstacle vertices " << gamma_sim_->getObstacleVertex(res));
        // res = gamma_sim_->getNextObstacleVertexNo(res);
        // ROS_INFO_STREAM("Number of obstacle vertices " << gamma_sim_->getObstacleVertex(res));
        // res = gamma_sim_->getNextObstacleVertexNo(res);
        // ROS_INFO_STREAM("Number of obstacle vertices " << gamma_sim_->getObstacleVertex(res));
    }
    gamma_sim_->processObstacles();
    ROS_INFO("Processed obstacles");

    for (auto &agent : agents_)
    {
        agent.first = gamma_sim_->addAgent(RVO::Vector2(agent.second.odom_.pose.pose.position.x, agent.second.odom_.pose.pose.position.y));
        if (agent.first == RVO::RVO_ERROR)
            ROS_INFO("Failed to add agent");
    }

    return true;
}

//Read scenario file to get all obstacle polygons
bool readObstacles(const std::string &file)
{
    // std::ifstream obs_file;
    // obs_file.open(file, std::ifstream::in);

    // if (obs_file.fail())
    //     return false;

    //Obstacles are polygons of varying vertices. Obstacle polygons in file are separated by a newline
    std::vector<RVO::Vector2> obs = {RVO::Vector2(0, 0), RVO::Vector2(5, 0), RVO::Vector2(5, 5), RVO::Vector2(0, 5)};
    obstacles_.push_back(obs);

    return true;
}

//Read scenario file to get all agents and their respective waypoints
bool readAgents(const std::string &file)
{
    // std::ifstream agent_file;
    // agent_file.open(file, std::ifstream::in);

    // if(agent_file.fail())
    //     return false;

    //Agents are

    //Based on waypoints previously read, which is a dictionary of name and position, set agent's list of waypoints
    AgentInfo temp_agent;
    geometry_msgs::Point point1, point2;
    point1.x = 6;
    point1.y = 6;
    point2.x = -1;
    point2.y = -1;
    temp_agent.waypoints_.push_back(point1);
    temp_agent.waypoints_.push_back(point2);
    temp_agent.odom_.pose.pose.position.x = -1;
    temp_agent.odom_.pose.pose.position.y = -1;
    agents_.push_back(std::make_pair(0, temp_agent));

    std::swap(temp_agent.waypoints_[0], temp_agent.waypoints_[1]);
    temp_agent.odom_.pose.pose.position.x = 6;
    temp_agent.odom_.pose.pose.position.y = 6;
    agents_.push_back(std::make_pair(0, temp_agent));
}

bool readWaypoints(const std::string &file)
{
    std::ifstream waypoints_file;
    waypoints_file.open(file, std::ifstream::in);

    if (waypoints_file.fail())
        return false;

    //Agents are
}

//Run simulation step by step
bool runStep()
{
    for (auto &agent_pair : agents_)
    {
        auto &agent = agent_pair.second;

        //Check if agent has reached waypoint
        agent.updateWaypoint();

        //Get preferred velocity of agent (straight line towards goal)
        auto wp = agent.waypoints_[agent.current_waypoint_];
        RVO::Vector2 current_waypoint(wp.x, wp.y);
        current_waypoint = RVO::normalize(current_waypoint - gamma_sim_->getAgentPosition(agent_pair.first));
        RVO::Vector2 pref_vel = current_waypoint * gamma_sim_->getAgentMaxSpeed(agent_pair.first);

        gamma_sim_->setAgentPrefVelocity(agent_pair.first, pref_vel);
    }

    ROS_INFO("To step");
    gamma_sim_->doStep();
    ROS_INFO("Stepped");

    //Update positions in local store
    for (auto &agent_pair : agents_)
    {
        auto &agent = agent_pair.second;
        RVO::Vector2 pos = gamma_sim_->getAgentPosition(agent_pair.first);
        agent.odom_.pose.pose.position.x = pos.x();
        agent.odom_.pose.pose.position.y = pos.y();
    }

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

bool publishPedestrianPosition()
{
    //Get velocity from GAMMA

    //Publish position to ROS

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamma_simulator");
    ros::NodeHandle nh;

    robot_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    agent_states_pub_ = nh.advertise<gamma_simulator::AgentStates>("/gamma_simulator/agent_states", 1);

    //Read ros paraams

    timeStep = 1 / rate;

    ROS_INFO("Read agents");
    readAgents("");
    readObstacles("");

    ROS_INFO("Setup GAMMA");
    setupGAMMA();

    ROS_INFO("Start GAMMA");

    ros::Rate r(rate);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        std::vector<RVO::Vector2> positions;
        for (const auto &id_pair : agents_)
            positions.push_back(gamma_sim_->getAgentPosition(id_pair.first));

        ROS_INFO_STREAM(positions[0].x() << ", " << positions[0].y() << "\t" << positions[1].x() << ", " << positions[1].y());
        // ROS_INFO_STREAM(gamma_sim_->getAgentPrefVelocity(agents_[0].first).x() << gamma_sim_->getAgentPrefVelocity(agents_[0].first).y());

        runStep();
    }

    return 0;
}