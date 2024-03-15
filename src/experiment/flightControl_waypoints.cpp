/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#include <iostream>
#include <cmath>
#include <vector>

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
    ros::init(argc, argv, "flightControl_waypoints");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/uav0/mavros/global_position/global", 10, globalPos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("/uav0/mavros/global_position/local", 10, localPos_cb);
    ros::Subscriber hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("/uav0/mavros/vfr_hud", 10, hud_cb);
    ros::Publisher init_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav0/mavros/cmd/takeoff");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
   
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "FCU Connection State: " << int(!current_state.connected) << std::endl;
    
    //send a few setpoints before starting
    geometry_msgs::PoseStamped localPos;
    ROS_INFO("Initializing...");
    for (int i = 10; ros::ok() && i > 0; i--){
        // prepare for switching to [OFFBOARD] mode
        localPos.pose.position.x = 10.0;
        localPos.pose.position.y = 0.0;
        localPos.pose.position.z = 50.0;
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

    std::cout << "Vehicle armed: " << int(!current_state.armed) << std::endl;

    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" &&
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

        localPos.pose.position.x = 10.0;
        localPos.pose.position.y = 0.0;
        localPos.pose.position.z = 50.0;
        local_pub.publish(localPos);
     
        ros::spinOnce();
        rate.sleep();
    }

    vector<vector<float>> waypoints{
        {30, 30, 50},
        {-30, 30, 50},
        {-0, -30, 50},
        {30, -30, 50}
    };
    int nav_point = 1;
    int index = 0;
    

    while(ros::ok()){
        index = nav_point-1;

        dist[0] = current_localPos.pose.pose.position.x - waypoints[index][0];
        dist[1] = current_localPos.pose.pose.position.y - waypoints[index][1];
        dist[2] = current_localPos.pose.pose.position.z - waypoints[index][2];
        threshold = sqrt(pow(dist[0], 2)+pow(dist[1], 2));
        
        if(threshold <= 10){
            nav_point++;
        }
        if(nav_point > 4){
            nav_point = 1;
        }

        localPos.pose.position.x = waypoints[index][0];
        localPos.pose.position.y = waypoints[index][1];
        localPos.pose.position.z = waypoints[index][2];
        local_pub.publish(localPos);

        cout << "Navigated Point: " << nav_point << endl;
     
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}