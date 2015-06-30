#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
//#include <sstream>
#include <string.h>
//#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/frames_io.hpp>
//#include <kdl/frames.hpp>
#include <map>
#include <kdl/chainiksolverpos_lma.hpp>
#include "sensor_msgs/JointState.h"
#include <math.h>

using namespace KDL;
using namespace std;

int useless;

int main ( int argc, char ** argv ){

   //inizializzo il nodo
   ros::init(argc, argv, "kinematics");

   //Tree che mi servirà per "prendere" il modello urdf
   KDL::Tree my_tree;

   //handle del nodo
   ros::NodeHandle node;

   //carico il modello urdf dal parameter server
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());

   //dall'urdf "costruisco" un Tree
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

   // Create the frame for the cartesian position
   KDL::Frame ik_cartpos,fk_cartpos,current_cartpos;

   //Create FKsolver based on kinematic tree
   TreeFkSolverPos_recursive fksolver = TreeFkSolverPos_recursive(my_tree);

   //I need a Chain for the inverse kinematics
   KDL::Chain my_chain;

   //let's tranform the Tree in a Chain
   my_tree.getChain( "base_link", "link6", my_chain );

   //parameter for IK
   //Eigen::Matrix<double,6,1> L; //(pare non serva)

   //declare the LMA solver for IK
   ChainIkSolverPos_LMA iksolver(my_chain);

   //joint variables in a JntArray
   unsigned int n = my_tree.getNrOfJoints();
   KDL::JntArray init_joint = JntArray(n);
   KDL::JntArray q_out = JntArray(n);
   KDL::JntArray q_out_fixed_pos = JntArray(n);
   KDL::JntArray q_out_fixed_orient = JntArray(n);
   KDL::JntArray q_current = JntArray(n);

   //initialisation joint array
   /*init_joint(0)=0;
   init_joint(1)=0;
   init_joint(2)=0;
   init_joint(3)=0;
   init_joint(4)=0;
   init_joint(5)=0;*/

   //variables for the cartesian target
   float x,y,z,x_vers,y_vers,z_vers,x_current,y_current,z_current,r1_x_current,r1_y_current,r1_z_current,r2_x_current,r2_y_current,r2_z_current,r3_x_current,r3_y_current,r3_z_current;
   float phi,theta,psi,theta_AA,theta_AA_last=0,theta_AA_current,theta_AA_incr;
   float R_init_des[3][3],R[3][3],R_base_att[3][3],R_base_des[3][3],r_AA[3],r_AA_current[3];

   ////////////////////PUBLISHER/////////////////
   ros::Publisher joint_state_pub = node.advertise<sensor_msgs::JointState>("j", 1000);

  //transmission rate
  ros::Rate loop_rate(10);

  //message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();
  msg.name.resize(6);
  msg.position.resize(6);

  //incremental variables
  int count = 0,init_flag=1,flag_last_position=0,flag_last_orientation=0;
  float incr=0.05,intorno=0.03, delta_trajectory_pos,delta_trajectory_orient;

  //distance from current pos and target pos
  float norma, norma_r1,norma_r2,norma_r3;

  while (ros::ok())
  {
    if( init_flag==1 ){ //initial position

        sleep(1);

    	//joint1
	msg.name[0] ="base_to_link1";
	msg.position[0]=0;

        //joint2
	msg.name[1] ="link1_to_link2";
	msg.position[1]=0;

	//joint3
	msg.name[2] ="link2_to_link3";
	msg.position[2]=-3.14/2;

	//joint4
	msg.name[3] ="link3_to_link4";
	msg.position[3]=0;

	//joint5
	msg.name[4] ="link4_to_link5";
	msg.position[4]=0;

	//joint6
	msg.name[5] ="link5_to_link6";
	msg.position[5]=0;

	init_joint(0)=msg.position[0];
	init_joint(1)=msg.position[1];
	init_joint(2)=msg.position[2];
	init_joint(3)=msg.position[3];
	init_joint(4)=msg.position[4];
	init_joint(5)=msg.position[5];

        q_out(0)=init_joint(0);
        q_out(1)=init_joint(1);
        q_out(2)=init_joint(2);
        q_out(3)=init_joint(3);
        q_out(4)=init_joint(4);
        q_out(5)=init_joint(5);

    	joint_state_pub.publish(msg);

    	init_flag=0;

    }else if (init_flag==0){

	    //acquisizione posizione cartesiana da tastiera
	    if(init_joint(0)==q_out(0) && init_joint(1)==q_out(1) && init_joint(2)==q_out(2) && init_joint(3)==q_out(3) && init_joint(4)==q_out(4) && init_joint(5)==q_out(5) ){

		fksolver.JntToCart(init_joint,fk_cartpos,"link6");

	    	cout << "\nPlease enter X value: ";
	    	cin >> x;
	    	cout << "Please enter Y value: ";
	    	cin >> y;
	    	cout << "Please enter Z value: ";
	    	cin >> z;

		// roll->ROTx , pitch->ROTy , yaw->ROTz
		cout << "\nInserire roll: ";
		cin >> psi;
		cout << "Inserire pitch: ";
		cin >> theta;
		cout << "Inserire yaw: ";
		cin >> phi;

		//calcolo matrice di rotazione corrispondente a rotazioni roll,pitch,yaw RISPETTO A TERNA BASE
		R[0][0] = cos(phi)*cos(theta); R[0][1] = -(sin(phi)*cos(psi))+(cos(phi)*sin(theta)*sin(psi));  R[0][2] = (sin(phi)*sin(psi))+(cos(phi)*sin(theta)*cos(psi));
		R[1][0] = sin(phi)*cos(theta); R[1][1] = (sin(phi)*sin(theta)*sin(psi)) + (cos(phi)*cos(psi)); R[1][2] = (sin(phi)*sin(theta)*cos(psi)) - (cos(phi)*sin(psi));
		R[2][0] = -sin(theta);         R[2][1] = cos(theta)*sin(psi);                                  R[2][2] = cos(theta)*cos(psi);

		//R_base_attuale TRASPOSTA !!!
	        R_base_att[0][0]=fk_cartpos.M.data[0]; R_base_att[0][1]=fk_cartpos.M.data[3]; R_base_att[0][2]=fk_cartpos.M.data[6];
	        R_base_att[1][0]=fk_cartpos.M.data[1]; R_base_att[1][1]=fk_cartpos.M.data[4]; R_base_att[1][2]=fk_cartpos.M.data[7];
	        R_base_att[2][0]=fk_cartpos.M.data[2]; R_base_att[2][1]=fk_cartpos.M.data[5]; R_base_att[2][2]=fk_cartpos.M.data[8];

		//calcolo matrice di rotazione corrispondente a rotazioni roll,pitch,yaw RISPETTO A TERNA CORRENTE (dipende dall'inizializzazione dei giunti <= j3=-90°)
		// OVVERO: trasp(R_base_att) * R_base_des
		R_init_des[0][0] = R_base_att[0][0]*R[0][0]+R_base_att[0][1]*R[1][0]+R_base_att[0][2]*R[2][0];  R_init_des[0][1] = R_base_att[0][0]*R[0][1]+R_base_att[0][1]*R[1][1]+R_base_att[0][2]*R[2][1];  R_init_des[0][2] = R_base_att[0][0]*R[0][2]+R_base_att[0][1]*R[1][2]+R_base_att[0][2]*R[2][2];
		R_init_des[1][0] = R_base_att[1][0]*R[0][0]+R_base_att[1][1]*R[1][0]+R_base_att[1][2]*R[2][0];  R_init_des[1][1] = R_base_att[1][0]*R[0][1]+R_base_att[1][1]*R[1][1]+R_base_att[1][2]*R[2][1];  R_init_des[1][2] = R_base_att[1][0]*R[0][2]+R_base_att[1][1]*R[1][2]+R_base_att[1][2]*R[2][2];
		R_init_des[2][0] = R_base_att[2][0]*R[0][0]+R_base_att[2][1]*R[1][0]+R_base_att[2][2]*R[2][0];  R_init_des[2][1] = R_base_att[2][0]*R[0][1]+R_base_att[2][1]*R[1][1]+R_base_att[2][2]*R[2][1];  R_init_des[2][2] = R_base_att[2][0]*R[0][2]+R_base_att[2][1]*R[1][2]+R_base_att[2][2]*R[2][2];

		ik_cartpos.M={R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]};

		ik_cartpos.p={ x,y,z };

		iksolver.CartToJnt( init_joint,ik_cartpos,q_out );

		//calcolo la norma del vettore errore di POSIZIONE
		norma = sqrt( ( (ik_cartpos.p[0]-fk_cartpos.p[0])*(ik_cartpos.p[0]-fk_cartpos.p[0]) + (ik_cartpos.p[1]-fk_cartpos.p[1])*(ik_cartpos.p[1]-fk_cartpos.p[1]) + (ik_cartpos.p[2]-fk_cartpos.p[2])*(ik_cartpos.p[2]-fk_cartpos.p[2]) ) );

		//calcolo il versore del vettore errore di POSIZIONE
		x_vers=(ik_cartpos.p[0]-fk_cartpos.p[0])/norma;
		y_vers=(ik_cartpos.p[1]-fk_cartpos.p[1])/norma;
		z_vers=(ik_cartpos.p[2]-fk_cartpos.p[2])/norma;

		//notazione asse-angolo
		theta_AA=acos((R_init_des[0][0]+R_init_des[1][1]+R_init_des[2][2]-1)/2);

		theta_AA_last=0;
		theta_AA_current=0;

		if(theta_AA<=0.001){

			theta_AA=0;

			r_AA[0]=1;
			r_AA[1]=0;
			r_AA[2]=0;
		}else{
			r_AA[0]=(1/(2*sin(theta_AA)))*(R_init_des[2][1]-R_init_des[1][2]);
			r_AA[1]=(1/(2*sin(theta_AA)))*(R_init_des[0][2]-R_init_des[2][0]);
			r_AA[2]=(1/(2*sin(theta_AA)))*(R_init_des[1][0]-R_init_des[0][1]);
		}

		float n_interpol;

		//delta incrementale della traiettoria lineare
		delta_trajectory_pos=0.04;
		n_interpol=norma/0.04;
		delta_trajectory_orient=theta_AA/n_interpol;

	    }

	    //calcolo frame cartesiano del TARGET corrente
	    x_current=fk_cartpos.p[0]+x_vers*delta_trajectory_pos;
	    y_current=fk_cartpos.p[1]+y_vers*delta_trajectory_pos;
	    z_current=fk_cartpos.p[2]+z_vers*delta_trajectory_pos;

	    //asse-angolo velore attuale
	    //theta_AA_last=acos((fk_cartpos.M.data[0]+fk_cartpos.M.data[4]+fk_cartpos.M.data[8]-1)/2);

	    if(theta_AA>theta_AA_last && theta_AA!=0){
		theta_AA_current=theta_AA_last+delta_trajectory_orient;
		theta_AA_incr=delta_trajectory_orient;
	    }
	    else if(theta_AA<theta_AA_last && theta_AA!=0){
		theta_AA_current=theta_AA_last-delta_trajectory_orient;
		theta_AA_incr=-delta_trajectory_orient;
	    }
	    else if(theta_AA==theta_AA_last){
		theta_AA_incr=0;
		theta_AA_current=theta_AA;
	    }

	    if( abs(theta_AA-theta_AA_current) <= delta_trajectory_orient ){
			//cout << "\n\n**METTO LA THETA FINALE!!!***\n\n";
			theta_AA_incr=theta_AA-theta_AA_last;
			theta_AA_current=theta_AA;
			//theta_AA_incr=0;
	    }

	    //ri-calcolo matrice di rotazione corrispondente al nuovo asse-angolo-target corrente
	    r1_x_current=((r_AA[0]*r_AA[0])*(1-cos(theta_AA_incr)))+cos(theta_AA_incr);
	    r1_y_current=r_AA[0]*r_AA[1]*(1-cos(theta_AA_incr))+r_AA[2]*sin(theta_AA_incr);
	    r1_z_current=r_AA[0]*r_AA[2]*(1-cos(theta_AA_incr))-r_AA[1]*sin(theta_AA_incr);

	    r2_x_current=r_AA[0]*r_AA[1]*(1-cos(theta_AA_incr))-r_AA[2]*sin(theta_AA_incr);
	    r2_y_current=((r_AA[1]*r_AA[1])*(1-cos(theta_AA_incr)))+cos(theta_AA_incr);
	    r2_z_current=r_AA[1]*r_AA[2]*(1-cos(theta_AA_incr))+r_AA[0]*sin(theta_AA_incr);

	    r3_x_current=r_AA[0]*r_AA[2]*(1-cos(theta_AA_incr))+r_AA[1]*sin(theta_AA_incr);
	    r3_y_current=r_AA[1]*r_AA[2]*(1-cos(theta_AA_incr))-r_AA[0]*sin(theta_AA_incr);
	    r3_z_current=((r_AA[2]*r_AA[2])*(1-cos(theta_AA_incr)))+cos(theta_AA_incr);

	    //RI-trasformo la matrice di rotazione rispetto alla BASE
	    R_base_att[0][0]=fk_cartpos.M.data[0]; R_base_att[0][1]=fk_cartpos.M.data[1]; R_base_att[0][2]=fk_cartpos.M.data[2];
	    R_base_att[1][0]=fk_cartpos.M.data[3]; R_base_att[1][1]=fk_cartpos.M.data[4]; R_base_att[1][2]=fk_cartpos.M.data[5];
	    R_base_att[2][0]=fk_cartpos.M.data[6]; R_base_att[2][1]=fk_cartpos.M.data[7]; R_base_att[2][2]=fk_cartpos.M.data[8];

	    R_base_des[0][0]=R_base_att[0][0]*r1_x_current+R_base_att[0][1]*r1_y_current+R_base_att[0][2]*r1_z_current; R_base_des[0][1]=R_base_att[0][0]*r2_x_current+R_base_att[0][1]*r2_y_current+R_base_att[0][2]*r2_z_current; R_base_des[0][2]=R_base_att[0][0]*r3_x_current+R_base_att[0][1]*r3_y_current+R_base_att[0][2]*r3_z_current;
	    R_base_des[1][0]=R_base_att[1][0]*r1_x_current+R_base_att[1][1]*r1_y_current+R_base_att[1][2]*r1_z_current; R_base_des[1][1]=R_base_att[1][0]*r2_x_current+R_base_att[1][1]*r2_y_current+R_base_att[1][2]*r2_z_current; R_base_des[1][2]=R_base_att[1][0]*r3_x_current+R_base_att[1][1]*r3_y_current+R_base_att[1][2]*r3_z_current;
	    R_base_des[2][0]=R_base_att[2][0]*r1_x_current+R_base_att[2][1]*r1_y_current+R_base_att[2][2]*r1_z_current; R_base_des[2][1]=R_base_att[2][0]*r2_x_current+R_base_att[2][1]*r2_y_current+R_base_att[2][2]*r2_z_current; R_base_des[2][2]=R_base_att[2][0]*r3_x_current+R_base_att[2][1]*r3_y_current+R_base_att[2][2]*r3_z_current;

	    if( abs( ik_cartpos.p[0] - x_current) <= delta_trajectory_pos ) x_current=ik_cartpos.p[0];
	    if( abs( ik_cartpos.p[1] - y_current) <= delta_trajectory_pos ) y_current=ik_cartpos.p[1];
	    if( abs( ik_cartpos.p[2] - z_current) <= delta_trajectory_pos ) z_current=ik_cartpos.p[2];

	    current_cartpos.p={ x_current,y_current,z_current };
	    current_cartpos.M={ R_base_des[0][0],R_base_des[0][1],R_base_des[0][2],R_base_des[1][0],R_base_des[1][1],R_base_des[1][2],R_base_des[2][0],R_base_des[2][1],R_base_des[2][2] };

	    //inverse kinematic for the current target
	    iksolver.CartToJnt( init_joint,current_cartpos,q_current );

	    if( (abs( ik_cartpos.p[0] - current_cartpos.p[0] ) <= delta_trajectory_pos) &&
		(abs( ik_cartpos.p[1] - current_cartpos.p[1] ) <= delta_trajectory_pos) &&
		(abs( ik_cartpos.p[2] - current_cartpos.p[2] ) <= delta_trajectory_pos) &&
		( abs(theta_AA-theta_AA_current) <= delta_trajectory_orient ) &&
		flag_last_position==1 && flag_last_orientation==1){

		/*printf("\n****");
		for( int i=0;i<6;i++ ){
		printf("\n j%d: %f \n",i+1,q_current(i));
		}
		printf("****\n");*/

		//joint1
		msg.name[0] ="base_to_link1";
		msg.position[0]=q_out(0);

		//joint2
		msg.name[1] ="link1_to_link2";
		msg.position[1]=q_out(1);

		//joint3
		msg.name[2] ="link2_to_link3";
		msg.position[2]=q_out(2);

		//joint4
		msg.name[3] ="link3_to_link4";
		msg.position[3]=q_out(3);

		//joint5
		msg.name[4] ="link4_to_link5";
		msg.position[4]=q_out(4);

		//joint6
		msg.name[5] ="link5_to_link6";
		msg.position[5]=q_out(5);

		// pubblico sul topic
		joint_state_pub.publish(msg);

		init_joint(0)=q_out(0);
		init_joint(1)=q_out(1);
		init_joint(2)=q_out(2);
		init_joint(3)=q_out(3);
		init_joint(4)=q_out(4);
		init_joint(5)=q_out(5);

	        /*printf("\n****");
	        for( int i=0;i<6;i++ ){
		    printf("\n j%d: %f \n",i+1,q_out(i));
	        }
	        printf("****\n");*/
	    }else{

		    /*printf("\n****");
		    for( int i=0;i<6;i++ ){
			printf("\n j%d: %f \n",i+1,q_current(i));
		    }
		    printf("****\n");*/

		    //joint1
		    msg.name[0] ="base_to_link1";
		    msg.position[0]=q_current(0);

		    //joint2
		    msg.name[1] ="link1_to_link2";
		    msg.position[1]=q_current(1);

		    //joint3
		    msg.name[2] ="link2_to_link3";
		    msg.position[2]=q_current(2);

		    //joint4
		    msg.name[3] ="link3_to_link4";
		    msg.position[3]=q_current(3);

		    //joint5
		    msg.name[4] ="link4_to_link5";
		    msg.position[4]=q_current(4);

		    //joint6
		    msg.name[5] ="link5_to_link6";
		    msg.position[5]=q_current(5);

		    // pubblico sul topic
		    joint_state_pub.publish(msg);

		init_joint(0)=q_current(0);
		init_joint(1)=q_current(1);
		init_joint(2)=q_current(2);
		init_joint(3)=q_current(3);
		init_joint(4)=q_current(4);
		init_joint(5)=q_current(5);

		if( ( abs(theta_AA-theta_AA_current) <= delta_trajectory_orient ) ){

			flag_last_position=1;
		}
		if( (abs( ik_cartpos.p[0] - current_cartpos.p[0] ) <= delta_trajectory_pos) &&
		    (abs( ik_cartpos.p[1] - current_cartpos.p[1] ) <= delta_trajectory_pos) &&
		    (abs( ik_cartpos.p[2] - current_cartpos.p[2] ) <= delta_trajectory_pos)){

			flag_last_orientation=1;
		}
	    }

	    fk_cartpos=current_cartpos;
	    theta_AA_last=theta_AA_current;
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    //sleep( 1 );
  }
   /////////////////////END_PUBLISHER////////////////////////////
 
   return 0;

}




