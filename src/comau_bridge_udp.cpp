/*
   comau_cartesian_state_publisher.cpp

   Developer: Daniele De Gregorio
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <iostream>
#include <math.h>
#include <pthread.h>
#include <signal.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <kdl/frames_io.hpp>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_kdl.h"

//Boost
#include <boost/thread.hpp>
#include <boost/date_time.hpp>


//
#include "lar_comau/ComauState.h"
#include "lar_comau/ComauCommand.h"
#include "ComauSmartSix.h"
#include "FIROrder2.h"

#include <ctime>

#include "lar_tool_utils/UDPNode.h"

#define COMMAND_UPDATE_JOINTS 666

using namespace std;
using namespace lar_tools;

struct MyUDPMessage{
        int command;
        long time;
        float payload[32];
};

ros::NodeHandle* nh;
ros::Publisher joints_publisher;
ros::Subscriber joints_state;
lar_tools::UDPNode* receive_node;
lar_tools::UDPNode* send_node;
MyUDPMessage send_message;
MyUDPMessage receive_message;



/** JOIN MESSAGE*/
sensor_msgs::JointState joint_state_current;
sensor_msgs::JointState joint_state_setpoint;


void updateJoints() {

  while(receive_node->isReady() && send_node->isReady() && nh->ok()){

      receive_node->receive((void*)&receive_message,sizeof(receive_message));
      std::cout << "Received: "<<receive_message.payload[0]<<std::endl;

  }
/*
    joint_msg.name.resize(7);
    joint_msg.name[0] = "lwr_0_joint";
    joint_msg.name[1] = "lwr_1_joint";
    joint_msg.name[2] = "lwr_2_joint";
    joint_msg.name[3] = "lwr_3_joint";
    joint_msg.name[4] = "lwr_4_joint";
    joint_msg.name[5] = "lwr_5_joint";
    joint_msg.name[6] = "lwr_6_joint";
    joint_msg.position.resize(7);

    for (int i = 0; i < 7; i++) {
        joint_msg.position[i] = 0;
    }

    while (nh->ok() && node->isReady()) {

        joints_publisher.publish(joint_msg);
        ros::spinOnce();
    }
    */
}

/**
 *
 */
void jointStateReceived( const sensor_msgs::JointState& msg ){
    joint_state_current = msg;
    send_message.command=-1;
    send_message.time = -1;
    std::cout <<msg<<std::endl;
    for(int i = 0; i < 6; i++){
      send_message.payload[i] = msg.position[i];

    }
    send_node->send((void *)&send_message,sizeof(send_message));

}


void initializeJointState(sensor_msgs::JointState& msg){
    msg.header.stamp = ros::Time::now();
    msg.name.resize(6);
    msg.position.resize(6);
    msg.name[0] ="base_to_link1";
    msg.name[1] ="link1_to_link2";
    msg.name[2] ="link2_to_link3";
    msg.name[3] ="link3_to_link4";
    msg.name[4] ="link4_to_link5";
    msg.name[5] ="link5_to_link6";
    msg.position[0]= 0.0f;
    msg.position[1]= 0.0f;
    msg.position[2]= -M_PI/2;
    msg.position[3]= 0.0f;
    msg.position[4]= 0.0f;
    msg.position[5]= 0.0f;
}

/** MAIN NODE **/
int
main(int argc, char** argv) {


    // Initialize ROS
    ros::init(argc, argv, "comau_bridge_udp");
    ROS_INFO("comau_bridge_udp node started...");
    nh = new ros::NodeHandle();

    /* UDP NODE*/
    ROS_INFO("UDP Node creating...");
    receive_node = new lar_tools::UDPNode(7750);
    send_node = new lar_tools::UDPNode("127.0.0.1",7751);

    ROS_INFO("UDP Node created!");

    /* NODES */
    joints_publisher = nh->advertise<sensor_msgs::JointState>("/lar_comau/comau_joint_state_publisher", 1);
    joints_state = nh->subscribe( "/lar_comau/comau_joint_states",1,jointStateReceived );
    initializeJointState(joint_state_setpoint);



    /* PUBLISHING THREAD */
    boost::thread updateTFsThread(updateJoints);

    // Spin
    while (nh->ok() && send_node->isReady()) {
        /* RECEIVE MESSAGE*/
        //ROS_INFO("Waiting message.");
        //node->receiveMessage(message);
        //ROS_INFO("Received/n command:%d\n  data:%f\n-----------\n", message.command, message.payload[0]);

        send_message.command=333;
        send_message.time = -1;
        float* arr = new float[6];
        for(int i = 0; i < 6; i++){
          send_message.payload[i] = i*33;
        }
        send_node->send((void*)&send_message,sizeof(send_message));


        joints_publisher.publish(joint_state_setpoint);


        ros::spinOnce();
    }

    ROS_INFO("Waiting for thread ends...");
    updateTFsThread.join();
}
