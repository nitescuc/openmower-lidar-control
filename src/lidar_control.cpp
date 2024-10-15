#include "ros/ros.h"
#include "xbot_msgs/RobotState.h"

#include <pigpiod_if2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/SetBool.h>

int pi;

int lidar_pin = 4;
#define LIDAR_STATE_ACTIVE 0
#define LIDAR_STATE_INACTIVE 1

bool control_lidar;
bool is_idle = true;
bool lidar_enabled = false;

bool setLidarEnabled(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    lidar_enabled = req.data;
    res.success = true;
    return true;
}


void robot_state_cb(const xbot_msgs::RobotState::ConstPtr &state)
{
    // set lidar active when robot is not in idle state
    if ((state->current_state == "IDLE") || (state->current_state == "UNDOCKING")) {
        if (control_lidar && gpio_read(pi, lidar_pin) == LIDAR_STATE_ACTIVE) {
            gpio_write(pi, lidar_pin, LIDAR_STATE_INACTIVE);
        }
        is_idle = true;
    } else {
        if (control_lidar && lidar_enabled && gpio_read(pi, lidar_pin) == LIDAR_STATE_INACTIVE) {
            gpio_write(pi, lidar_pin, LIDAR_STATE_ACTIVE);
        }
        is_idle = false;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "lidar_control");
    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    paramNh.param("control_lidar", control_lidar, true);

    // Init GPIO
    pi = pigpio_start(NULL, NULL);
    set_mode(pi, lidar_pin, PI_OUTPUT);

    ros::ServiceServer lidar_enable_service = n.advertiseService("lidar_control/enable_lidar", setLidarEnabled);
    // Handle ROS communication events
    ros::Subscriber remote = n.subscribe("/xbot_monitoring/robot_state", 1, robot_state_cb);   

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    pigpio_stop(pi);

    return 0;
}