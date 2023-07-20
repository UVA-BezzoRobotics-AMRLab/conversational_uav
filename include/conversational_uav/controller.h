#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <map>
#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>

#define TAKEOFF_HEIGHT 1.3
#define GOAL_TOLERANCE 3e-1
#define MAX_V 1.0

enum State
{
    SITTING,
    TAKING_OFF,
    HOVER,
    LANDING,
    MOVING
};

struct Cmd
{
    int cmd_id;
    std::string cmd;
    std::string goal;

    friend std::ostream& operator<<(std::ostream& os, const Cmd& cmd) {
        os << "ID: " << cmd.cmd_id << std::endl;
        os << "Command: " << cmd.cmd << std::endl;
        if (!cmd.goal.empty()) {
            os << "Goal: " << cmd.goal << std::endl;
        }
        return os;
    }

}; typedef struct Cmd cmd_t;

class Controller {
public:
    Controller(ros::NodeHandle& nh);
    ~Controller();

    void spin();
    void pollFile(const ros::TimerEvent&);
    void controlLoop(const ros::TimerEvent&);
    
private:

    std::vector<std::string> valid_commands = {"TAKEOFF", "LAND", "TAKE_PICTURE", "GO_TO"};
    std::map<std::string, Eigen::Vector3d> valid_locations{
        {"A", Eigen::Vector3d(1.0,3.2,2)},
        {"B", Eigen::Vector3d(2, 0.5, 0.7)}
    };

    ros::Timer file_timer, control_timer;
    ros::Subscriber odom_sub;
    ros::Subscriber img_sub;

    ros::Publisher wp_pub;

    State state;

    Eigen::Vector3d odom;
    Eigen::Vector4d odom_yaw;
    Eigen::Vector3d goal;

    cmd_t command;
    bool is_new_cmd, is_initialized;
    int prev_cmd_id;

    sensor_msgs::CompressedImage::ConstPtr img;

    std::vector<std::string> splitStringByTabs(const std::string& input);
    bool testCommandStr(const std::vector<std::string>& components);

    // callbacks
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
    void imgcb(const sensor_msgs::CompressedImage::ConstPtr& msg);
};

#endif
