#pragma once

#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

#include "jointmpc.h"
#include <cmath>
#include <vector>

class PlannerNode
{
public:
    PlannerNode(ros::NodeHandle* nodehandle);

    void run(Mpc &Opt, std::ofstream &logFile);

    ros::NodeHandle nh_;

    void dataLogger(std::ofstream &logFile, std::vector<double> &plan);    

private:
    ros::Subscriber ego_pos_;
    ros::Subscriber ego_vel_;
    ros::Subscriber ego_acc_;
    ros::Subscriber NV_pos_;
    ros::Subscriber NV_vel_;
    ros::Subscriber NV_acc_;
    ros::Subscriber obs_pos_;

    ros::Publisher plan_pos_;
    ros::Publisher plan_vel_;
    ros::Publisher plan_acc_;
    ros::Publisher plan_ua_;
    ros::Publisher plan_ul_;

    void initSubscribers();
    void initPublishers();

    void posSubscriberCallback(const geometry_msgs::Pose::ConstPtr &pos_msg);
    void velSubscriberCallback(const geometry_msgs::Twist::ConstPtr &vel_msg);
    void accSubscriberCallback(const geometry_msgs::Accel::ConstPtr &acc_msg);
    void NVposSubscriberCallback(const geometry_msgs::Pose::ConstPtr &nv_pos_msg);
    void NVvelSubscriberCallback(const geometry_msgs::Twist::ConstPtr &nv_vel_msg);
    void NVaccSubscriberCallback(const geometry_msgs::Twist::ConstPtr &nv_acc_msg);
    void obsPosSubscriberCallback(const geometry_msgs::Pose::ConstPtr &obs_pos_msg);

    double lane_width = 3.6;
    double gap = 10.0;
    double infn = 1e6;

    // states
    double s_;
    double v_;
    double a_;
    double l_;
    double rl_;

    // NV
    double s_NV_;    
    double v_NV_;
    double a_NV_;
    double l_NV_;

    std::vector<double> s_NV_pred_;
    std::vector<double> s1_1_front_;
    std::vector<double> s1_1_rear_;
    std::vector<double> s1_2_front_;
    std::vector<double> s1_2_rear_;

    double s_obs_;
};