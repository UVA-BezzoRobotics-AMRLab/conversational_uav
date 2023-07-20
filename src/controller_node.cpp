#include <thread>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <conversational_uav/controller.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "conversation_uav_controller");
    ros::NodeHandle nh;

    Controller controller(nh);

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

    unsigned int i = 0;
    while(i <= 10 && !unpaused){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause/physics", srv);
        ++i;
    }

    if (!unpaused){
        ROS_ERROR("Could not wake up Gazebo");
        return -1;
    }

    ros::Duration(5.0).sleep();

    controller.spin();

    return 0;
}
