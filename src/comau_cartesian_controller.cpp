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

#include "lar_comau/ComauState.h"
#include "lar_comau/ComauCommand.h"
#include "ComauSmartSix.h"
#include "FIROrder2.h"

#include <ctime>


/**
 * Commons
 */
lar_comau::FIROrder2* fir;
lar_comau::ComauSmartSix* robot;
sensor_msgs::JointState last_msg;


/**
 * Reset Fir with Target Pose
 */
void resetFIR(lar_comau::FIROrder2* fir,geometry_msgs::Pose& pose){
        float* initial_data = new float[6];
        initial_data[0] = pose.position.x;
        initial_data[1] = pose.position.y;
        initial_data[2] = pose.position.z;
        initial_data[3] = pose.orientation.x;
        initial_data[4] = pose.orientation.y;
        initial_data[5] = pose.orientation.z;
        initial_data[6] = pose.orientation.w;
        fir->setInitialData(initial_data);
}

/**
 * Callback for Pose Received
 */
geometry_msgs::Pose current_pose;
lar_comau::ComauCommand current_comau_command;
bool current_command_reset = false;
void poseReceived( const lar_comau::ComauCommand& comau_command ){

        current_comau_command = comau_command;
        std::string command = comau_command.command;
        geometry_msgs::Pose pose = comau_command.pose;

        std::cout << "Received Command:\n"<<comau_command<<std::endl;

        if(command.compare("close")==0) {
                exit(1);
        }

        if(command.compare("linear")==0) {
                current_command_reset = true;
                current_pose = pose;
        }else if(command.compare("joint")==0) {
                current_command_reset = true;
                current_pose = pose;
                resetFIR(fir,pose);
        }else{

        }

}

/**
 * Builds JointState message from q vector
 */
void buildJointStateForComau(sensor_msgs::JointState& msg, float* q){
        msg.header.stamp = ros::Time::now();
        msg.name.resize(6);
        msg.position.resize(6);
        msg.name[0] ="base_to_link1";
        msg.position[0]=q[0];
        msg.name[1] ="link1_to_link2";
        msg.position[1]=q[1];
        msg.name[2] ="link2_to_link3";
        msg.position[2]=q[2];
        msg.name[3] ="link3_to_link4";
        msg.position[3]=q[3];
        msg.name[4] ="link4_to_link5";
        msg.position[4]=q[4];
        msg.name[5] ="link5_to_link6";
        msg.position[5]=q[5];
}

/**
 * Compare two Joint State Messages
 */
bool compareJointStateMessages(sensor_msgs::JointState& msg1,sensor_msgs::JointState& msg2){
        if(msg1.position.size()<= 0 || msg2.position.size()<=0)
                return false;

        bool cond = true;
        for(int i = 0; i < 6; i++) {
                cond = cond && (msg1.position[i]==msg2.position[i]);
        }
        return cond;
}


/**
 * Callback for Joint State
 */
float * q_state = new float[6];
float* q_out = new float[6];
float q_distance = 0.0f;
bool joints_state_ready = false;
bool is_moving = false;
float moving_error_th = 0.0001f;
void jointStateReceived( const sensor_msgs::JointState& msg ){

        for(int i = 0; i < 6; i++) {
                q_state[i] = msg.position[i]* M_PI / 180.0f;
        }

        if(joints_state_ready==false) {
                current_comau_command.command = "joint";
                current_command_reset = true;
                joints_state_ready = true;
        }


        q_distance = 0.0f;
        for(int i = 0; i < 6; i++)
                q_distance = sqrt((q_state[i]-q_out[i])*(q_state[i]-q_out[i]));

        is_moving = q_distance > moving_error_th;
}


void broadcastComauTransforms(  tf::TransformBroadcaster& tf_broadcaster,lar_comau::ComauState& sending_pose){

        //END EFFECTOR
        tf::Transform t0U;
        t0U.setOrigin( tf::Vector3(
                               sending_pose.pose.position.x/1000.0f,
                               sending_pose.pose.position.y/1000.0f,
                               sending_pose.pose.position.z/1000.0f
                               ));

        tf::Quaternion q(
                sending_pose.pose.orientation.x,
                sending_pose.pose.orientation.y,
                sending_pose.pose.orientation.z,
                sending_pose.pose.orientation.w
                );

        t0U.setRotation(q);
        tf_broadcaster.sendTransform(tf::StampedTransform(t0U, ros::Time::now(), "base", "comau_t0U"));


        //BASE MARKER
        tf::Transform tBM;
        tf::poseKDLToTF (robot->base_marker, tBM);

        std::cout << "Sending:" <<tBM.getOrigin()[2]<<std::endl;
        tf_broadcaster.sendTransform(tf::StampedTransform(tBM, ros::Time::now(), "base", "comau_base_marker"));





}

int main(int argc, char *argv[])
{

        std::string node_name = "comau_cartesian_controller";

        //NODES
        ros::init(argc, argv, node_name);
        ros::NodeHandle n;
        tf::TransformBroadcaster tf_broadcaster;
        ros::Subscriber comau_cartesian_controller_sub = n.subscribe( "lar_comau/comau_cartesian_controller",1,poseReceived );
        ros::Subscriber comau_joint_states_sub = n.subscribe( "lar_comau/comau_joint_states",1,jointStateReceived );
        ros::Publisher comau_joint_state_pub = n.advertise<sensor_msgs::JointState>("lar_comau/comau_joint_state_publisher", 100);
        ros::Publisher comau_full_state_pub = n.advertise<lar_comau::ComauState>("lar_comau/comau_full_state_publisher", 1);
        std::cout << node_name << "... Initialized!"<<std::endl;

        //JOINT CONTROL MOVEMENT



        //ROBOT
        std::string robot_desc_string;
        n.param("robot_description", robot_desc_string, std::string());
        robot = new lar_comau::ComauSmartSix(robot_desc_string,"base_link", "link6");
        //TODO: Set tool by parameters
        //robot->setTool(0.04f,0.075f,-0.266f,0 , PI, 0); //TOOL CON LA PENNA

        robot->setTool(0.0f,0.0f,-0.095f,0 , PI, 0); // FLANGIA

        // robot->setTool(-0.1192f,0.0f,-0.095f-0.262f-0.025f,0, -PI/2.0f, 0); // FLANGIA

        //robot->setTool(0,0,0,0,0,0); // WRIST

        robot->setBaseMarker(0.151f,0.0f,-0.450f-0.100f,0.0f,PI/2.0f,-PI/2.0);


        //FIR 2-Order
        fir = new lar_comau::FIROrder2(7,0.0005);
        fir->setSampleTime(1);
        //Z -450

        bool first_iteration = true;

        while (ros::ok())
        {
                if(joints_state_ready) {

                        if(first_iteration) {
                                /**
                                 * First Iteration Reset Filter to current Robot Pose. It avoids initial spikes
                                 */

                                first_iteration = false;

                                float x,y,z,e1,e2,e3,qx,qy,qz,qw;

                                robot->fk(q_state,x,y,z,e1,e2,e3);
                                robot->fk(q_state,x,y,z,qx,qy,qz,qw);

                                current_pose.position.x = x*1000.0f;
                                current_pose.position.y = y*1000.0f;
                                current_pose.position.z = z*1000.0f;
                                current_pose.orientation.x = qx;
                                current_pose.orientation.y = qy;
                                current_pose.orientation.z = qz;
                                current_pose.orientation.w = qw;

                                resetFIR(fir,current_pose);

                        }else{

                                /**
                                 * Initial Security Check for IK solution available or not
                                 */
                                int c = robot->ik(
                                        current_pose.position.x,
                                        current_pose.position.y,
                                        current_pose.position.z,
                                        current_pose.orientation.x,
                                        current_pose.orientation.y,
                                        current_pose.orientation.z,
                                        current_pose.orientation.w,
                                        q_state,
                                        q_out
                                        );


                                if(c<0) {
                                        /**
                                         * No IK Solution
                                         */
                                        std::cout << "Not Reachable!!"<<std::endl;
                                        current_command_reset = false;

                                }else{

                                        fir->setInput(0,current_pose.position.x);
                                        fir->setInput(1,current_pose.position.y);
                                        fir->setInput(2,current_pose.position.z);
                                        fir->setInput(3,current_pose.orientation.x);
                                        fir->setInput(4,current_pose.orientation.y);
                                        fir->setInput(5,current_pose.orientation.z);
                                        fir->setInput(6,current_pose.orientation.w);

                                        int c;

                                        /**
                                         * Cheks Movement Type. If JOINT send IK only one time. If LINEAR sends IK solution over each iteration
                                         */
                                        bool new_q_out = false;
                                        if(current_comau_command.command.compare("joint")==0 && current_command_reset) {
                                                std::cout << "New REF!!"<<std::endl;
                                                c = robot->ik(
                                                        fir->output[0],
                                                        fir->output[1],
                                                        fir->output[2],
                                                        fir->output[3],
                                                        fir->output[4],
                                                        fir->output[5],
                                                        fir->output[6],
                                                        q_state,
                                                        q_out
                                                        );
                                                new_q_out = true;
                                        }else if(current_comau_command.command.compare("linear")==0) {
                                                std::cout << "New REF!!"<<std::endl;
                                                c = robot->ik(
                                                        fir->output[0],
                                                        fir->output[1],
                                                        fir->output[2],
                                                        fir->output[3],
                                                        fir->output[4],
                                                        fir->output[5],
                                                        fir->output[6],
                                                        q_state,
                                                        q_out
                                                        );
                                                new_q_out = true;
                                        }else{
                                                std::cout << "Same REF!!"<<std::endl;
                                        }

                                        /**
                                         * Builds Joint State Command to send
                                         */
                                        if(new_q_out) {
                                                sensor_msgs::JointState msg;
                                                buildJointStateForComau(msg,q_out);
                                                comau_joint_state_pub.publish(msg);
                                                last_msg=msg;

                                        }
                                        // Reset Command as Consumed
                                        current_command_reset = false;
                                        ros::spinOnce();


                                        lar_comau::ComauState sending_pose;
                                        sending_pose.moving = is_moving;
                                        float x,y,z,e1,e2,e3,qx,qy,qz,qw;
                                        robot->fk(q_state,x,y,z,qx,qy,qz,qw);

                                        sending_pose.pose.position.x = x*1000.0f;
                                        sending_pose.pose.position.y = y*1000.0f;
                                        sending_pose.pose.position.z = z*1000.0f;
                                        sending_pose.pose.orientation.x = qx;
                                        sending_pose.pose.orientation.y = qy;
                                        sending_pose.pose.orientation.z = qz;
                                        sending_pose.pose.orientation.w = qw;

                                        sending_pose.q.resize(6);
                                        for(int i = 0; i < 6; i++) {
                                                sending_pose.q[i] = q_state[i];
                                        }
                                        comau_full_state_pub.publish(sending_pose);
                                        broadcastComauTransforms(tf_broadcaster,sending_pose);

                                        /**
                                         * Debug
                                         */

                                        std::cout << "Filtered Pose:\n";
                                        std::cout << "x: "<<fir->output[0]<<std::endl;
                                        std::cout << "y: "<<fir->output[1]<<std::endl;
                                        std::cout << "z: "<<fir->output[2]<<std::endl;
                                        std::cout << "qx: "<<fir->output[3]<<std::endl;
                                        std::cout << "qy: "<<fir->output[4]<<std::endl;
                                        std::cout << "qz: "<<fir->output[5]<<std::endl;
                                        std::cout << "qw: "<<fir->output[6]<<std::endl;
                                        std::cout << "Filtered Qs:\n";
                                        for(int i = 0; i < 6; i++)
                                                std::cout << "j"<<i+1<<": "<<q_out[i]<<std::endl;

                                        if(is_moving) {
                                                std::cout << "Robot is moving!! "<< q_distance<<std::endl;
                                        }else{
                                                std::cout << "Robot is standing!! "<<std::endl;
                                        }
                                }
                                std::system("clear");

                        }

                }else{
                        std::cout << "Waiting for robot state..\n";
                        std::system("clear");
                }


                ros::spinOnce();
        }
        return 0;
}
