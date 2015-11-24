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
#include <boost/date_time/posix_time/posix_time.hpp>

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

struct MyUDPMessage {
        int command;
        long time;
        float payload[32];
};

struct RobotState {
        float* q;
        float* q_dot;
        geometry_msgs::Pose pose;
        geometry_msgs::Pose pose_3;

        RobotState(){
                q = new float[6];
                q_dot = new float[6];
        }
};

/** NODES */
ros::NodeHandle* nh;
ros::Publisher joints_publisher;
ros::Publisher cartesian_publisher;
ros::Subscriber joints_state;
tf::TransformBroadcaster* tf_broadcaster;
lar_tools::UDPNode* receive_node;
lar_tools::UDPNode* send_node;
MyUDPMessage send_message;
MyUDPMessage receive_message;

/** ROBOT */
lar_comau::ComauSmartSix* robot=NULL;
RobotState current_robot_state;
float* pose_temp_data = new float[7];

/** JOIN MESSAGE*/
sensor_msgs::JointState joint_state_current;
sensor_msgs::JointState joint_state_setpoint;
bool feedback_ready = false;


bool activeCondition(){
    return receive_node->isReady() && send_node->isReady() && nh->ok();
}

/**
 * Feedback from GUI
 */
void receiveFromGui() {

        while(activeCondition()) {

                receive_node->receive((void*)&receive_message,sizeof(receive_message));
                for(int i = 0; i < 6; i++) {
                        joint_state_setpoint.position[i] = receive_message.payload[i];
                }
                //std::cout << "Received: "<<joint_state_setpoint<<std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
}


/**
 * Broadcast TFs
 */
void broadcastTFs(){

        while(activeCondition()) {

                tf::Transform t06;
                lar_tools::geometrypose_to_tf(current_robot_state.pose,t06);
                tf_broadcaster->sendTransform(tf::StampedTransform(t06, ros::Time::now(), "base", "comau_t06"));


                tf::Transform t0BM;
                tf::poseKDLToTF (robot->base_marker, t0BM);
                tf_broadcaster->sendTransform(tf::StampedTransform(t0BM, ros::Time::now(), "base", "comau_t_0_base_marker"));

                boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }

}

/**
 *
 */
void jointStateReceived( const sensor_msgs::JointState& msg ){
        if(robot==NULL) {
                ROS_INFO("Error: Robot not initialized!");
        }


        joint_state_current = msg;

        send_message.command=333;
        send_message.time = -1;
        for(int i = 0; i < 6; i++) {
                send_message.payload[i] = msg.position[i];
                send_message.payload[i+10] = msg.velocity[i];
                current_robot_state.q[i] = msg.position[i]*M_PI/180.0f;
                current_robot_state.q_dot[i] = msg.velocity[i];
        }

        //Forward Kinematics
        robot->fk(current_robot_state.q,current_robot_state.pose);

        //Send state to GUI
        //ROS_INFO("Message command %d!!",send_message.command);
        send_node->send((void *)&send_message,sizeof(send_message));
        if(feedback_ready==false) {
                ROS_INFO("Feedback from comau ready!!");
                //joint_state_setpoint = joint_state_current;

        }
        feedback_ready = true;
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
        msg.position[2]= -90.0f;
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

        /* PARAMS */

        /* ROBOT */
        std::string robot_desc_string;
        nh->param("robot_description", robot_desc_string, std::string());
        robot = new lar_comau::ComauSmartSix(robot_desc_string,"base_link", "link6");
        robot->setBaseMarker(0.155f+0.008f,0.0f,-0.450f-0.100f-0.001f,0.0f,PI/2.0f,-PI/2.0);

        /* UDP NODE*/
        ROS_INFO("UDP Node creating...");
        receive_node = new lar_tools::UDPNode(7750);
        send_node = new lar_tools::UDPNode("127.0.0.1",7751);
        if(!receive_node->isReady() || !send_node->isReady() ) {
                ROS_INFO("UDP ERROR!");
        }else{
                ROS_INFO("UDP Node created!");
        }

        /* NODES */
        joints_publisher = nh->advertise<sensor_msgs::JointState>("/lar_comau/comau_joint_state_publisher", 1);
        joints_state = nh->subscribe( "/lar_comau/comau_joint_states",1,jointStateReceived );
        cartesian_publisher = nh->advertise<lar_comau::ComauState>("lar_comau/comau_full_state_publisher", 1);
        tf_broadcaster= new tf::TransformBroadcaster;
        initializeJointState(joint_state_setpoint);



        /* PUBLISHING THREAD */
        boost::thread receiveFromGuiThread(receiveFromGui);
        boost::thread broadcastTFsThread(broadcastTFs);

        ros::Rate rate(2);
        // Spin
        while (activeCondition()) {
                /* RECEIVE MESSAGE*/
                //ROS_INFO("Waiting message.");
                //node->receiveMessage(message);
                //ROS_INFO("Received/n command:%d\n  data:%f\n-----------\n", message.command, message.payload[0]);




                if(feedback_ready) {

                        //FK

                        //std::cout << "Robot pose:\n"<<current_robot_state.pose<<std::endl;
                        //std::cout << "Robot pose3:\n"<<current_robot_state.pose_3<<std::endl;

                        std::cout << "llop\n";
                        joints_publisher.publish(joint_state_setpoint);
                }

                ros::spinOnce();
                rate.sleep();
        }

        ROS_INFO("Waiting for thread ends...");
        receiveFromGuiThread.join();
        broadcastTFsThread.join();
}
