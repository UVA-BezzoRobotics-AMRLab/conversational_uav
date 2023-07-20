#include <sstream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include <conversational_uav/controller.h>

#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

Controller::Controller(ros::NodeHandle& nh){

    file_timer = nh.createTimer(ros::Duration(1), &Controller::pollFile, this);
    control_timer = nh.createTimer(ros::Duration(.1), &Controller::controlLoop, this);

    odom_sub = nh.subscribe("/firefly/ground_truth/odometry", 1, &Controller::odomcb, this);
    img_sub = nh.subscribe("/firefly/vi_sensor/camera_depth/camera/image_raw/compressed", 1, &Controller::imgcb, this);

    wp_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>
            ("/firefly/command/trajectory", 10);

    is_new_cmd = false;
    is_initialized = false;

    command.cmd_id = -1;
    prev_cmd_id = -1;

    state = SITTING;

}

Controller::~Controller(){
    
}

void Controller::spin(){

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
}

void Controller::odomcb(const nav_msgs::Odometry::ConstPtr& msg){

    tf::Quaternion q(
	    msg->pose.pose.orientation.x,
	    msg->pose.pose.orientation.y,
	    msg->pose.pose.orientation.z,
	    msg->pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

    odom = Eigen::Vector3d(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);

    odom_yaw = Eigen::Vector4d(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z,
                           yaw);

    if (!is_initialized){
        is_initialized = true;
        ROS_INFO("Controller initialized!");
    }
}

void Controller::imgcb(const sensor_msgs::CompressedImage::ConstPtr& msg){
    img = msg;
}

void Controller::controlLoop(const ros::TimerEvent&){

    if (!is_new_cmd || !is_initialized)
        return;

    if (command.cmd == "TAKE_PICTURE"){
        try {
            // Convert the CompressedImage to an OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);

            // Save the image to a file (you can replace "/path/to/save/image.png" with your desired file path)
            std::string filepath = "/home/nick/uav_img.png";
            cv::imwrite(filepath, cv_ptr->image);

            ROS_INFO("Image saved to: %s", filepath.c_str());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        command.cmd = "";

        return;
    }

    switch(state){
        case SITTING:

            if (command.cmd == "TAKEOFF"){
                ROS_INFO("Taking off!");
                // takeoff quadrotor
                trajectory_msgs::MultiDOFJointTrajectory msg;
                msg.header.stamp = ros::Time::now();
                msg.joint_names.push_back("base_link");

                trajectory_msgs::MultiDOFJointTrajectoryPoint point;

                goal = odom;
                goal(2) += TAKEOFF_HEIGHT;

                point.transforms.resize(1);
                point.transforms[0].translation.x = goal[0];
                point.transforms[0].translation.y = goal[1];
                point.transforms[0].translation.z = goal[2];

                point.transforms[0].rotation.w = 1.0;

                point.velocities.resize(1);
                point.accelerations.resize(1);
                point.time_from_start = ros::Duration(5.);

                msg.points.push_back(point);

                wp_pub.publish(msg);
                command.cmd = "";
                state = TAKING_OFF;
            }
            
        break;
        case TAKING_OFF:
            if ((odom-goal).norm() < GOAL_TOLERANCE){
                ROS_INFO("in hover state");
                state = HOVER;
            }
        break;
        case HOVER:
            if (command.cmd == "LAND"){
                ROS_INFO("Landing!");

                // land quadrotor
                trajectory_msgs::MultiDOFJointTrajectory msg;
                msg.header.stamp = ros::Time::now();
                msg.joint_names.push_back("base_link");

                trajectory_msgs::MultiDOFJointTrajectoryPoint point;

                goal = odom;
                goal(2) = 0;

                point.transforms.resize(1);
                point.transforms[0].translation.x = goal[0];
                point.transforms[0].translation.y = goal[1];
                point.transforms[0].translation.z = goal[2];

                point.transforms[0].rotation.w = 1.0;

                point.velocities.resize(1);
                point.accelerations.resize(1);
                point.time_from_start = ros::Duration(5.);

                msg.points.push_back(point);

                wp_pub.publish(msg);
                command.cmd = "";
                state = LANDING;
            } else if(command.cmd == "GO_TO"){
                ROS_INFO("Going to location!");

                // land quadrotor
                trajectory_msgs::MultiDOFJointTrajectory msg;
                msg.header.stamp = ros::Time::now();
                msg.joint_names.push_back("base_link");

                trajectory_msgs::MultiDOFJointTrajectoryPoint point;

                goal = valid_locations[command.goal];

                point.transforms.resize(1);
                point.transforms[0].translation.x = goal[0];
                point.transforms[0].translation.y = goal[1];
                point.transforms[0].translation.z = goal[2];

                point.transforms[0].rotation.w = 1.0;

                point.velocities.resize(1);
                point.accelerations.resize(1);
                
                double t = (goal-odom).norm()/MAX_V;
                point.time_from_start = ros::Duration(t);

                msg.points.push_back(point);

                wp_pub.publish(msg);
                command.cmd = "";
                state = MOVING;
            } else if (command.cmd == "TAKE_PICTURE"){

            }
        break;
        case LANDING:
            if ((odom-goal).norm() < GOAL_TOLERANCE){
                ROS_INFO("in sitting state");
                state = SITTING;
            }
        break;
        case MOVING:
            if ((odom-goal).norm() < GOAL_TOLERANCE){
                ROS_INFO("in hover state");
                state = HOVER;
            }
        break;
        default:

        break;
    }

}

void Controller::pollFile(const ros::TimerEvent&){

    std::string filename = "/home/nick/uav_cmds.txt";
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Error opening file: " << filename);
        return;
    }

    std::string firstLine;
    if (std::getline(file, firstLine)){
        std::vector<std::string> components = splitStringByTabs(firstLine);

        if (!testCommandStr(components))
            return;


        // shouldn't ever fail because this was tested in testCommandStr
        try{
            command.cmd_id = std::stoi(components[0]);
        } catch (const std::exception& e){
            ROS_ERROR_STREAM("This should never happen...");
            return;
        }

        command.cmd = components[1];
        if (components.size() >= 3)
            command.goal = components[2];
        else
            command.goal = "";
        
        ROS_INFO_STREAM("New command is:\n" << command);

        is_new_cmd = true;
        prev_cmd_id = command.cmd_id;

    }

}

bool Controller::testCommandStr(const std::vector<std::string>& components){
    if (components.size() < 2){
        ROS_ERROR("Not enough tokens in command!");
        return false;
    }

    int id;
    try{
        id = std::stoi(components[0]);
    } catch (const std::exception& e){
        ROS_ERROR_STREAM("Error parsing command from file: " << e.what());
        ROS_ERROR_STREAM("Is command id an integer?");
        return false;
    }

    if (id == prev_cmd_id && prev_cmd_id != -1)
        return false;

    if (components.size() > 3)
        ROS_WARN("Warning: Too many tokens in command, only looking at first 3");

    if (components.size() >= 3){

        if(valid_locations.find(components[2]) == valid_locations.end()){
            ROS_ERROR("Invalid location provided");
            return false;
        }

    }

    if(std::find(valid_commands.begin(), valid_commands.end(), components[1]) != valid_commands.end())
        return true;
    else{
        ROS_ERROR_STREAM("Invalid command: " << components[1]);
        return false;
    }
    
}

std::vector<std::string> Controller::splitStringByTabs(const std::string& input) {
    std::vector<std::string> tokens;
    std::istringstream stream(input);
    std::string token;

    while (std::getline(stream, token, '\t')) {
        tokens.push_back(token);
    }

    return tokens;
}

