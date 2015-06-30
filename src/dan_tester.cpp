#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include "ComauSmartSix.h"
#include "geometry_msgs/Pose.h"
#include "lar_comau/ComauPose.h"
#include "lar_comau/ComauCommand.h"

using namespace std;


//#include <sstream>

float * q_call = new float[6];
bool first_callback = false;
void state_callback( const sensor_msgs::JointState& msg ){

  for(int i = 0; i < 6 ; i++){
    q_call[i] = msg.position[i] ;
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
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("lar_comau/comau_joint_state_publisher", 1000);
  ros::Publisher cartesian_controller = n.advertise<lar_comau::ComauCommand>("lar_comau/comau_cartesian_controller", 1000);

  ros::Rate loop_rate(10);

  //robot
  std::string robot_desc_string;
  n.param("robot_description", robot_desc_string, std::string());
  lar_comau::ComauSmartSix robot(robot_desc_string,"base_link", "link6");


  //ROS_INFO("\n\n%s\n\n",argv[1]);

  int count = 0;
  while (ros::ok())
  {


    sensor_msgs::JointState msg;
    geometry_msgs::Pose pose;


    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();
    std::string command;

    cout << "Please enter pose value: ";
    cin >> command;
    cin >> x;
    cin >> y;
    cin >> z;
    cin >> roll;
    cin >> pitch;
    cin >> yaw;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;


    KDL::Rotation rot = KDL::Rotation::RPY(
      roll*M_PI/180.0f,
      pitch*M_PI/180.0f,
      yaw*M_PI/180.0f
    );

    double qx,qy,qz,qw;
    rot.GetQuaternion(qx,qy,qz,qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;

    lar_comau::ComauCommand comau_command;
    comau_command.pose = pose;
    comau_command.command = command;

    cartesian_controller.publish(comau_command);
    ros::spinOnce();

    /*
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    KDL::JntArray j_q_in = KDL::JntArray(6);
    KDL::JntArray j_q_out = KDL::JntArray(6);
    float * q_in = new float[6];
    float * q_out = new float[6];


    for(int i =0; i < 6; i++){
        q_in[i] = q_call[i] * PI / 180.0f;

    }


    int c = robot->ik(x,y,z,roll,pitch,yaw,q_in,q_out);
    //int c = 1;

    q_out[0] = x * M_PI / 180.0f;
    q_out[1] = y * M_PI / 180.0f;
    q_out[2] = z * M_PI / 180.0f;
    q_out[3] = roll * M_PI / 180.0f;
    q_out[4] = pitch * M_PI / 180.0f;
    q_out[5] = yaw * M_PI / 180.0f;


    for(int i =0; i < 6; i++){
        std::cout << "q_in " <<i << " "<<q_in[i]<< " -> ";
        std::cout << "q_out " <<i << " "<<q_out[i]<<std::endl;
    }

    if(c<0){
      std::cout << "Somethings went wrong...";
    }else{
        msg.header.stamp = ros::Time::now();
        msg.name.resize(6);
        msg.position.resize(6);
        msg.name[0] ="base_to_link1";
        msg.position[0]=q_out[0];
        msg.name[1] ="link1_to_link2";
        msg.position[1]=q_out[1];
        msg.name[2] ="link2_to_link3";
        msg.position[2]=q_out[2];
        msg.name[3] ="link3_to_link4";
        msg.position[3]=q_out[3];//abs(q_out[3]>PI)?q_out[3]/PI: q_out[3];
        msg.name[4] ="link4_to_link5";
        msg.position[4]=q_out[4];
        msg.name[5] ="link5_to_link6";
        msg.position[5]=q_out[5];//abs(q_out[5]>PI)?q_out[5]/PI: q_out[5];

        std::cout << "MSG:\n" << msg<<std::endl;

        joint_state_pub.publish(msg);
    }
    }else{
      std::cout << "wait"<<std::endl;
    }




    ++count;
    */
  }


  return 0;
}
