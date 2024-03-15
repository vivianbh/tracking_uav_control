/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>

using namespace std;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_globalPos;
nav_msgs::Odometry current_localPos;
mavros_msgs::VFR_HUD current_hud;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void globalPos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_globalPos = *msg;
}

void localPos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_localPos = *msg;
}

void hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_hud = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fw_takeoff");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, globalPos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, localPos_cb);
    ros::Subscriber hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10, hud_cb);
    ros::Publisher init_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
   
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "FCU Connection State: " << current_state.connected << std::endl;
    
    //send a few setpoints before starting
    geometry_msgs::PoseStamped localPos;
    ROS_INFO("Initializing...");
    for (int i = 10; ros::ok() && i > 0; i--){
        // prepare for switching to [OFFBOARD] mode
        localPos.pose.position.x = 10.0;
        localPos.pose.position.y = 0.0;
        localPos.pose.position.z = 30.0;
        local_pub.publish(localPos);
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = current_globalPos.latitude;
    takeoff_cmd.request.longitude = current_globalPos.longitude + 0.008;
    takeoff_cmd.request.altitude = current_globalPos.altitude + 30;
    bool is_takeoff = 0;
    
    ros::Time last_request = ros::Time::now();
    float flightHeight = 0;
    float alt_ini = current_hud.altitude;
    float dist[3], threshold=100;
    bool flag = 0;

    while(ros::ok()){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        if( current_state.armed && !is_takeoff){
            takeoff_client.call(takeoff_cmd);
            ROS_INFO("Takeoff");
            is_takeoff = takeoff_cmd.response.success;
        }

        flightHeight = current_hud.altitude - alt_ini;
        if( is_takeoff && current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent ){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        if(current_state.mode == "OFFBOARD"){
            break;
        }

        dist[0] = current_localPos.pose.pose.position.x - localPos.pose.position.x;
        dist[1] = current_localPos.pose.pose.position.y - localPos.pose.position.y;
        dist[2] = current_localPos.pose.pose.position.z - localPos.pose.position.z;

        threshold = sqrt(pow(dist[0], 2)+pow(dist[1], 2));
/*
        if(threshold > 5 && !flag){
            localPos.pose.position.x = 0.0;
            localPos.pose.position.y = 0.0;
            localPos.pose.position.z = 30.0;
            local_pub.publish(localPos);
        } 
*/      
        localPos.pose.position.x = 700.0;
        localPos.pose.position.y = 0.0;
        localPos.pose.position.z = 30.0;
        local_pub.publish(localPos);
     
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        localPos.pose.position.x = 700.0;
        localPos.pose.position.y = 0.0;
        localPos.pose.position.z = 50.0;
        local_pub.publish(localPos);
     
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}