/* 
    Viranjan Bhattacharyya
    EMC2 Lab Clemson University
*/

#include "include/node3.h"
// #include "include/jointmpc.h"
// #include "include/imputer.h"
#include <cstdlib>
#include <fstream>

PlannerNode::PlannerNode(ros::NodeHandle *nodehandle) : 
    nh_(*nodehandle),
    s_(0.1), v_(0.1), a_(0.1), l_(1.0), rl_(0.1),
    s_NV_(150.0), v_NV_(0.0), a_NV_(0.0), l_NV_(2.0),
    alpha_v(50.0), alpha_a(50.0),
    s_obs_(150.0)
{
    initSubscribers();
    initPublishers();
}

void PlannerNode::initSubscribers()
{
    // ROS_INFO("initializing subscribers");
    ego_pos_ = nh_.subscribe("/ego_pos_topic", 10, &PlannerNode::posSubscriberCallback, this);
    ego_vel_ = nh_.subscribe("/ego_vel_topic", 10, &PlannerNode::velSubscriberCallback, this);
    ego_acc_ = nh_.subscribe("/ego_acc_topic", 10, &PlannerNode::accSubscriberCallback, this);
    NV_pos_ = nh_.subscribe("/NV_pos_topic", 10, &PlannerNode::NVposSubscriberCallback, this);
    NV_vel_ = nh_.subscribe("/NV_vel_topic", 10, &PlannerNode::NVvelSubscriberCallback, this);
    NV_acc_ = nh_.subscribe("/NV_acc_topic", 10, &PlannerNode::NVaccSubscriberCallback, this);
    obs_pos_ = nh_.subscribe("/obs_pos_topic", 10, &PlannerNode::obsPosSubscriberCallback, this);
}

void PlannerNode::initPublishers()
{
    // ROS_INFO("initializing publishers");
    plan_pos_ = nh_.advertise<geometry_msgs::Pose>("/plan_pos_topic", 10, this);
    plan_vel_ = nh_.advertise<geometry_msgs::Twist>("/plan_vel_topic", 10, this);
    plan_acc_ = nh_.advertise<geometry_msgs::Accel>("/plan_acc_topic", 10, true);
    plan_ua_ = nh_.advertise<geometry_msgs::Accel>("/plan_ua_topic", 10, true);
    plan_ul_ = nh_.advertise<geometry_msgs::Pose>("/plan_ul_topic", 10, true);
}

void PlannerNode::posSubscriberCallback(const geometry_msgs::Pose::ConstPtr &pos_msg)
{
    s_ = pos_msg->position.x;
    l_ = pos_msg->position.y;
    // ROS_INFO("planner received s: %f, l: %f", s_, l_);
}

void PlannerNode::velSubscriberCallback(const geometry_msgs::Twist::ConstPtr &vel_msg)
{
    v_ = vel_msg->linear.x;
    rl_ = vel_msg->linear.y;
    // ROS_INFO("planner received v: %f, rl: %f", v_, rl_);
}

void PlannerNode::accSubscriberCallback(const geometry_msgs::Accel::ConstPtr &acc_msg)
{
    a_ = acc_msg->linear.x;
    // ROS_INFO("planner received a: %f", a_);
}

void PlannerNode::NVposSubscriberCallback(const geometry_msgs::Pose::ConstPtr &nv_msg)
{
    double x_NV = nv_msg->position.x;
    double y_NV = nv_msg->position.y;
    // convert received position of NV in simulator to hl frame
    s_NV_ = -((x_NV - 85.235) * cos(0.003895) + (y_NV - 13.415) * sin(0.003895));
    l_NV_ = (-(x_NV - 85.235) * sin(0.003895) + (y_NV - 13.415) * cos(0.003895)) / lane_width + 1;
    // ROS_INFO("s_NV: %f, l_NV: %f", s_NV_, l_NV_);
}

void PlannerNode::NVvelSubscriberCallback(const geometry_msgs::Twist::ConstPtr &nv_vel_msg)
{
    double vx = nv_vel_msg->linear.x;
    double vy = nv_vel_msg->linear.y;
    v_NV_ = sqrt(vx * vx + vy * vy);
}

void PlannerNode::NVaccSubscriberCallback(const geometry_msgs::Twist::ConstPtr &nv_acc_msg)
{
    double ax = nv_acc_msg->linear.x;
    double ay = nv_acc_msg->linear.y;
    a_NV_ = sqrt(ax * ax + ay * ay);
}

void PlannerNode::obsPosSubscriberCallback(const geometry_msgs::Pose::ConstPtr &obs_pos_msg) 
{
    double x_obs = obs_pos_msg->position.x;
    double y_obs = obs_pos_msg->position.y;
    s_obs_ = -((x_obs - 85.235) * cos(0.003895) + (y_obs - 13.415) * sin(0.003895));
}

void PlannerNode::run(Imputer &Cost, Mpc &Opt, std::ofstream &logFile)
{
    int k = 0; // sim step counter
    std::vector<std::vector<double>> ego_data;
    std::vector<std::vector<double>> nv_data;
    ros::Rate rate(20); // Hz
    while (ros::ok()) {

        // aiMPC
        std::vector<double> X0;
        X0.push_back(s_);
        X0.push_back(v_);
        X0.push_back(a_);
        X0.push_back(l_);
        X0.push_back(rl_);
        
        ego_data.push_back(X0);

        std::vector<double> X0_NV;
        X0_NV.push_back(s_NV_);
        X0_NV.push_back(v_NV_);
        X0_NV.push_back(a_NV_);

        nv_data.push_back(X0_NV);

        geometry_msgs::Pose plan_pos;
        geometry_msgs::Twist plan_vel;
        geometry_msgs::Accel plan_acc;
        geometry_msgs::Accel plan_ua;
        geometry_msgs::Pose plan_ul;        
        
        if (k == (r - 1)) {
            // need to pass last r step traj data
            std::vector<double> alphas = Cost.impute(nv_data, ego_data);

            alpha_v = alphas[0];
            alpha_a = alphas[1];

            ego_data.clear();
            nv_data.clear();
        }
        std::vector<double> plan = Opt.sol(X0, X0_NV, s_obs_, alpha_v, alpha_a);

        plan_pos.position.x = plan[0]; // s
        plan_pos.position.y = plan[3]; // l
        plan_vel.linear.x = plan[1]; // v
        plan_vel.linear.y = plan[4]; // rl
        plan_acc.linear.x = plan[2]; // a
        plan_ua.linear.x = plan[5]; // u_a
        plan_ul.position.y = plan[6]; // u_l

        this->dataLogger(logFile, plan);

        plan_pos_.publish(plan_pos);
        plan_vel_.publish(plan_vel);
        plan_acc_.publish(plan_acc);
        plan_ua_.publish(plan_ua);
        plan_ul_.publish(plan_ul);

        ros::spinOnce();
        rate.sleep();

        if (k == (r - 1)) {k = 0;}
        else {k++;}
        std::cout << "data step: " << k << std::endl;
    }
}

void PlannerNode::dataLogger(std::ofstream &logFile, std::vector<double> &plan) {
    
    if (logFile.is_open()) {
        logFile << s_ << " " << v_ << " " << a_ << " " << l_ << " " << rl_ << " 11111 " << " "
        << plan[0] << " " << plan[1] << " " << plan[2] << " " << plan[3] << " " << plan[4] << " " << plan[5] << " " << plan[6] << " "
        << "11111" << " " << s_NV_ << " " << v_NV_ << " " << a_NV_ << " " << l_NV_ << " " << "11111" << " " << s_obs_ << " " << "11111" << " " << alpha_v << " " << alpha_a << std::endl;
    }
    else std::cout << "Unable to open file" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Planner_Node");

    ros::NodeHandle nh;

    Mpc Opt; // instantiate MPC object

    Imputer Cost; // instantiate Imputer object

    std::ofstream logFile ("src/expt3/data/log.txt"); // declare log file

    PlannerNode planner_node(&nh);

    planner_node.run(Cost, Opt, logFile);

    ROS_INFO("Bye...");

    return 0;   
}