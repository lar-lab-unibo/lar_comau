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


#define COMMAND_SEND_JOINTS 333
#define COMMAND_SEND_CARTESIAN 777
#define COMMAND_RECEIVED_JOINTS 333
#define COMMAND_RECEIVED_CARTESIAN 777
#define COMMAND_RECEIVED_CARTESIAN_TARGET 778

#define OP_MODE_JOINTS 10001
#define OP_MODE_CARTESIAN 10005
#define OP_MODE_TARGET 10010


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
        int op_mode;

        RobotState(){
                q = new float[6];
                q_dot = new float[6];
                op_mode = OP_MODE_JOINTS;
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

/* CAMERA */
bool use_camera = false;
Eigen::Matrix4d T_6_CAMERA;

/** TARGET */
bool got_target = false;
Eigen::Matrix4d T_0_TARGET;
Eigen::Matrix4d T_0_TARGETAPPROACH;

/** JOIN MESSAGE*/
sensor_msgs::JointState joint_state_current;
sensor_msgs::JointState joint_state_setpoint;
geometry_msgs::Pose pose_setpoint;
sensor_msgs::JointState joint_state_setpoint_ik;
bool feedback_ready = false;


bool activeCondition(){
        return receive_node->isReady() && send_node->isReady() && nh->ok();
}

/**
 * Feedback from GUI
 */
void receiveFromGui() {

        while(activeCondition()) {
                //RESET VARIABLES
                //got_target = false;

                //RECEIVE
                receive_node->receive((void*)&receive_message,sizeof(receive_message));
                ROS_INFO("Received Message with Command: %d",receive_message.command);
                //
                if(receive_message.command==COMMAND_RECEIVED_JOINTS) {
                        ROS_INFO("Received Joints Command from GUI");
                        current_robot_state.op_mode = OP_MODE_JOINTS;
                        for(int i = 0; i < 6; i++) {
                                joint_state_setpoint.position[i] = receive_message.payload[i];
                        }
                }
                if(receive_message.command==COMMAND_RECEIVED_CARTESIAN) {
                        ROS_INFO("Received Cartesian Command from GUI");

                        current_robot_state.op_mode = OP_MODE_CARTESIAN;
                        double rx,ry,rz,rroll,rpitch,ryaw;
                        rx =   receive_message.payload[0];
                        ry =   receive_message.payload[1];
                        rz =   receive_message.payload[2];
                        rroll =   receive_message.payload[3] *M_PI/180.0;
                        rpitch =   receive_message.payload[4] *M_PI/180.0;
                        ryaw =   receive_message.payload[5] *M_PI/180.0;
                        xyzrpy_to_geometrypose_d(
                                rx,ry,rz,
                                rroll,rpitch,ryaw,
                                pose_setpoint
                                );
                }
                if(receive_message.command==COMMAND_RECEIVED_CARTESIAN_TARGET) {
                        ROS_INFO("Received Cartesian Target from GUI %f %f %f %f %f ",
                                 receive_message.payload[0],
                                 receive_message.payload[1],
                                 receive_message.payload[2],
                                 receive_message.payload[3],
                                 receive_message.payload[4]
                               );

                        T_0_TARGET = Eigen::Matrix4d::Identity();
                        T_0_TARGET(0,3) = receive_message.payload[0];
                        T_0_TARGET(1,3) = receive_message.payload[1];
                        T_0_TARGET(2,3) = receive_message.payload[2];
                        double azimuth = receive_message.payload[3];
                        double zenith = receive_message.payload[4];
                        double roll = receive_message.payload[5];
                        double distance = receive_message.payload[6]/1000.0;
                        double send = receive_message.payload[7];
                        Eigen::Matrix4d T_TARGET_AZI,T_TARGET_ZENI,T_TARGET_ROLL,T_TARGET_DISTANCE;
                        lar_tools::create_eigen_4x4_d(0,0,0,0,zenith*M_PI/180.0,0,T_TARGET_ZENI);
                        lar_tools::create_eigen_4x4_d(0,0,0,0,0,azimuth*M_PI/180.0,T_TARGET_AZI);
                        lar_tools::create_eigen_4x4_d(0,0,0,0,0,roll*M_PI/180.0,T_TARGET_ROLL);
                        lar_tools::create_eigen_4x4_d(0,0,distance,0,0,0,T_TARGET_DISTANCE);
                        T_0_TARGETAPPROACH = T_0_TARGET*T_TARGET_AZI;
                        T_0_TARGETAPPROACH = T_0_TARGETAPPROACH*T_TARGET_ZENI;
                        T_0_TARGETAPPROACH = T_0_TARGETAPPROACH*T_TARGET_DISTANCE;
                        T_0_TARGETAPPROACH = T_0_TARGETAPPROACH*T_TARGET_ROLL;
                        if(send>0){
                          eigen_4x4_to_geometrypose_d(T_0_TARGETAPPROACH,pose_setpoint);
                          pose_setpoint.position.x *=1000.0;
                          pose_setpoint.position.y *=1000.0;
                          pose_setpoint.position.z *=1000.0;

                        }
                        got_target = true;
                }
                //std::cout << "Received: "<<joint_state_setpoint<<std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
}

/**
 * Feedback from GUI
 */
void sendToGui() {

        while(activeCondition()) {

                if(feedback_ready) {
                        /** SENDs GUI JOINT MESSAGE */
                        send_message.command=COMMAND_SEND_JOINTS;
                        send_message.time = -1;
                        for(int i = 0; i < 6; i++) {
                                send_message.payload[i] = joint_state_current.position[i];
                                send_message.payload[i+10] = joint_state_current.velocity[i];
                                current_robot_state.q[i] = joint_state_current.position[i]*M_PI/180.0f;
                                current_robot_state.q_dot[i] = joint_state_current.velocity[i];
                        }
                        send_node->send((void *)&send_message,sizeof(send_message));

                        /** SENDs GUI CO§ARTESIAN MESSAGE */
                        send_message.command=COMMAND_SEND_CARTESIAN;
                        send_message.time = -1;

                        double sx,sy,sz,sroll,spitch,syaw;
                        lar_tools::geometrypose_to_xyzrpy_d(
                                current_robot_state.pose,
                                sx,sy,sz,
                                sroll,spitch,syaw
                                );
                        send_message.payload[0]=sx*1000.0;
                        send_message.payload[1]=sy*1000.0;
                        send_message.payload[2]=sz*1000.0;
                        send_message.payload[3] =sroll*180.0/M_PI;
                        send_message.payload[4]=spitch*180.0/M_PI;
                        send_message.payload[5]=syaw*180.0/M_PI;
                        send_node->send((void *)&send_message,sizeof(send_message));
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
}

/**
 * Broadcast TFs
 */
void broadcastTFs(){

        while(activeCondition()) {
                if(feedback_ready) {
                        //POSE PUBLISHER
                        cartesian_publisher.publish(current_robot_state.pose);

                        //WRIST TF
                        tf::Transform t06;
                        lar_tools::geometrypose_to_tf(current_robot_state.pose,t06);
                        tf_broadcaster->sendTransform(tf::StampedTransform(t06, ros::Time::now(), "base", "comau_t06"));

                        //BASE MARKER TF
                        tf::Transform t0BM;
                        tf::poseKDLToTF (robot->base_marker, t0BM);
                        tf_broadcaster->sendTransform(tf::StampedTransform(t0BM, ros::Time::now(), "base", "comau_t_0_base_marker"));

                        if(use_camera) {
                                tf::Transform t6camera;
                                lar_tools::eigen_4x4_d_to_tf(T_6_CAMERA,t6camera);
                                tf_broadcaster->sendTransform(tf::StampedTransform(t6camera, ros::Time::now(), "comau_t06", "comau_t_6_camera"));
                        }

                        if(got_target){
                            tf::Transform t0target,t0targetapproach;
                            lar_tools::eigen_4x4_d_to_tf(T_0_TARGET,t0target);
                            lar_tools::eigen_4x4_d_to_tf(T_0_TARGETAPPROACH,t0targetapproach);
                            tf_broadcaster->sendTransform(tf::StampedTransform(t0target, ros::Time::now(), "base", "comau_t0target"));
                            tf_broadcaster->sendTransform(tf::StampedTransform(t0targetapproach, ros::Time::now(), "base", "comau_t0targetapproach"));
                        }
                }
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
        //Forward Kinematics
        robot->fk(current_robot_state.q,current_robot_state.pose);

        if(feedback_ready==false) {
                ROS_INFO("Feedback from comau ready!!");
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
        nh = new ros::NodeHandle("");

        /* CAMERA */
        use_camera = true;
        // Grabby Gripper Camera
        // Note that YAW value is opposed due to DETECTION ERROR //TODO: find why?
        lar_tools::create_eigen_4x4_d(0.061,-0.0094,-0.1488,179.0 * M_PI/180.0f,0,89.5 * M_PI/180.0f,T_6_CAMERA);

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
        cartesian_publisher = nh->advertise<geometry_msgs::Pose>("lar_comau/comau_full_state_publisher", 1);
        tf_broadcaster= new tf::TransformBroadcaster;


        initializeJointState(joint_state_setpoint);
        initializeJointState(joint_state_setpoint_ik);



        /* PUBLISHING THREAD */
        boost::thread receiveFromGuiThread(receiveFromGui);
        boost::thread sendToGuiThread(sendToGui);
        boost::thread broadcastTFsThread(broadcastTFs);

        ros::Rate rate(60);
        //TODO: Manage Send Speed
        int robot_update_limiter = 30;
        int robot_update_limiter_counter = 0;
        // Spin
        while (activeCondition()) {
                /* RECEIVE MESSAGE*/
                //ROS_INFO("Waiting message.");
                //node->receiveMessage(message);
                //ROS_INFO("Received/n command:%d\n  data:%f\n-----------\n", message.command, message.payload[0]);




                if(feedback_ready) {


                        robot_update_limiter_counter++;
                        if(robot_update_limiter_counter>=robot_update_limiter) {

                                if(current_robot_state.op_mode == OP_MODE_JOINTS) {
                                        joints_publisher.publish(joint_state_setpoint);
                                        ROS_INFO("Real robot update set point! JOINT_MODE");
                                }
                                else if(current_robot_state.op_mode == OP_MODE_CARTESIAN) {
                                        float* q_out = new float[6];



                                        std::cout << pose_setpoint<<std::endl;
                                        int c = robot->ik(
                                                pose_setpoint.position.x,
                                                pose_setpoint.position.y,
                                                pose_setpoint.position.z,
                                                pose_setpoint.orientation.x,
                                                pose_setpoint.orientation.y,
                                                pose_setpoint.orientation.z,
                                                pose_setpoint.orientation.w,
                                                current_robot_state.q,
                                                q_out
                                                );
                                        if(c>=0) {
                                                for(int i =0; i < 6; i++)
                                                        joint_state_setpoint_ik.position[i] = q_out[i]*180.0/M_PI;

                                                joints_publisher.publish(joint_state_setpoint_ik);
                                                ROS_INFO("Real robot update set point! CARTESIAN_MODE");


                                        }else{
                                                ROS_INFO("IK FAILED!");
                                        }
                                }





                                robot_update_limiter_counter = 0;


                        }

                }

                ros::spinOnce();
                rate.sleep();
        }

        ROS_INFO("Waiting for thread ends...");
        receiveFromGuiThread.join();
        sendToGuiThread.join();
        broadcastTFsThread.join();
}
