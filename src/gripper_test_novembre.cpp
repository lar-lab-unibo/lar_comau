#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include "ComauSmartSix.h"
#include "geometry_msgs/Pose.h"
#include "lar_comau/ComauCommand.h"

using namespace std;

struct MyPose {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
};

std::vector<MyPose> my_poses;

void initPoses(){

        float h = 50;
        float h_top = 200;
        float h_table = -50;
        float h_table_top = 20;
        float approach_h_top = 50;
        float y = 0;
        float y_top = -110;
        float x = 980;
        float x_delta = 200;
        float approach_delta = 100;



        MyPose side1;
        side1.x=x; side1.y=y; side1.z=h;
        side1.roll=90; side1.pitch=90; side1.yaw=0;

        MyPose side2;
        side2.x=x; side2.y=y+approach_delta; side2.z=h_table;
        side2.roll=90; side2.pitch=90; side2.yaw=0;

        MyPose side3;
        side3.x=x; side3.y=y; side3.z=h_table;
        side3.roll=90; side3.pitch=90; side3.yaw=0;

        MyPose side4;
        side4.x=x+x_delta; side4.y=y; side4.z=h;
        side4.roll=90; side4.pitch=170; side4.yaw=0;

        MyPose side5;
        side5.x=x-x_delta; side5.y=y; side5.z=h;
        side5.roll=90; side5.pitch=20; side5.yaw=0;



        MyPose top1;
        top1.x=980; top1.y=500; top1.z=-120;
        top1.roll=90; top1.pitch=180; top1.yaw=0;

        MyPose top2;
        top2.x=980; top2.y=20; top2.z=-170;
        top2.roll=90; top2.pitch=180; top2.yaw=0;

        MyPose top3;
        top3.x=x; top3.y=y_top; top3.z=h_top;
        //top3.roll=0; top3.pitch=90; top3.yaw=0;
        top3.roll=0; top3.pitch=180; top3.yaw=170;


        MyPose off;
        off.x=x; off.y=y_top; off.z=500;
        off.roll=0; off.pitch=180; off.yaw=0;

        MyPose port;
        port.x=1000; port.y=-800; port.z=100;
        port.roll=0; port.pitch=90; port.yaw=-40;

        my_poses.push_back(side1);
        my_poses.push_back(side2);
        my_poses.push_back(side3);
        my_poses.push_back(side4);
        my_poses.push_back(side5);

        my_poses.push_back(top1);
        my_poses.push_back(top2);
        my_poses.push_back(top3);

        my_poses.push_back(off);
        my_poses.push_back(port);
}

//#include <sstream>

float * q_call = new float[6];
bool first_callback = false;
void state_callback( const sensor_msgs::JointState& msg ){

        for(int i = 0; i < 6; i++) {
                q_call[i] = msg.position[i];
        }
        std::cout << msg.position[3]<<std::endl;
        first_callback = true;

}

int main(int argc, char **argv)
{

        float x=0.0f;
        float y=0.0f;
        float z=0.0f;
        float roll=0.0f;
        float pitch=0.0f;
        float yaw=0.0f;

        ros::init(argc, argv, "dan_tester");

        ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe( "lar_comau/comau_joint_states",1,state_callback );
        ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("lar_comau/comau_joint_state_publisher", 1);
        ros::Publisher cartesian_controller = n.advertise<lar_comau::ComauCommand>("lar_comau/comau_cartesian_controller", 1);

        ros::Rate loop_rate(10);

        //robot
        std::string robot_desc_string;
        n.param("robot_description", robot_desc_string, std::string());
        lar_comau::ComauSmartSix robot(robot_desc_string,"base_link", "link6");


        //ROS_INFO("\n\n%s\n\n",argv[1]);
        initPoses();
        int count = 0;
        while (ros::ok())
        {


                sensor_msgs::JointState msg;
                geometry_msgs::Pose pose;


                //std::stringstream ss;
                //ss << "hello world " << count;
                //msg.data = ss.str();
                std::string command;
                int command_index;


                cout << "Please enter pose value: ";
                cin >> command_index;
                if(command_index<my_poses.size() && command_index>=0) {
                        std::cout << "OK command "<<command_index<<std::endl;
                        MyPose mypose = my_poses[command_index];
                        std::cout << mypose.x << "," <<mypose.y<<std::endl;
                        pose.position.x = mypose.x;
                        pose.position.y = mypose.y;
                        pose.position.z = mypose.z;

                        KDL::Rotation rot = KDL::Rotation::RPY(
                                mypose.roll*M_PI/180.0f,
                                mypose.pitch*M_PI/180.0f,
                                mypose.yaw*M_PI/180.0f
                                );

                        double qx,qy,qz,qw;
                        rot.GetQuaternion(qx,qy,qz,qw);
                        pose.orientation.x = qx;
                        pose.orientation.y = qy;
                        pose.orientation.z = qz;
                        pose.orientation.w = qw;

                        lar_comau::ComauCommand comau_command;
                        comau_command.pose = pose;
                        comau_command.command = "joint";

                        cartesian_controller.publish(comau_command);
                        ros::spinOnce();
                        std::cout << "Published "<<comau_command<<std::endl;
                }





        }


        return 0;
}
