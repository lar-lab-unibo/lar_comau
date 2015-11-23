/*
 * File:   ComauSmartSix.cpp
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */

#include "ComauSmartSix.h"

namespace lar_comau {

ComauSmartSix::ComauSmartSix(
        std::string robot_description,
        std::string base_link,
        std::string tip_link
        ) {

        this->robot_description = robot_description;

        /** Create TREE */
        if (!kdl_parser::treeFromString(robot_description, this->tree)) {
                ROS_ERROR("Failed to construct kdl tree");
        }else{
                this->tree_solver_fk = new TreeFkSolverPos_recursive(this->tree);

                /** Create CHAIN */
                this->tree.getChain( base_link, tip_link, this->chain );

                chain_solver_fk = new ChainFkSolverPos_recursive(this->chain);
                chain_solver_vel_ik = new ChainIkSolverVel_pinv(this->chain);

                /** Joints LIMITS */
                int n = this->tree.getNrOfJoints();
                this->q_limit_max = JntArray(n);
                this->q_limit_min = JntArray(n);

                /** TODO: pass limits with Params */
                q_limit_max(0)=2.9670;
                q_limit_max(1)=2.7052;
                q_limit_max(2)=0;
                q_limit_max(3)=3.6651; //3.14;//3.6651;
                q_limit_max(4)=2.2689; //1.7444;//2.2689;
                q_limit_max(5)=6.28; //47.1;

                q_limit_min(0)=-2.9670;
                q_limit_min(1)=-1.4835;
                q_limit_min(2)=-2.9670;
                q_limit_min(3)=-3.6651; //-3.14;//-3.6651;
                q_limit_min(4)=-2.2689; //-1.7444;//-2.2689;
                q_limit_min(5)=-6.28; //-47.1;

                /** Refine Sovlers */
                chain_solver_ik_jl = new ChainIkSolverPos_NR_JL(
                        this->chain,
                        q_limit_min,
                        q_limit_max,
                        *chain_solver_fk,
                        *chain_solver_vel_ik
                        );

                chain_solver_ik_lma = new ChainIkSolverPos_LMA(this->chain);

                /** Tool */
                this->tool.p = {0,0,0};
                this->tool.M = KDL::Rotation::EulerZYZ(0, 0, 0);
        }
}

ComauSmartSix::~ComauSmartSix() {

}

void ComauSmartSix::setTool(float x, float y, float z, float e1, float e2, float e3,std::string angle_type){
        this->tool.p = {x,y,z};
        this->tool.M = KDL::Rotation::EulerZYZ(e1,e2,e3);
}

void ComauSmartSix::setTool(geometry_msgs::Pose& pose){
        tf::PoseMsgToKDL(pose,this->tool);
}

void ComauSmartSix::setBaseMarker(float x, float y, float z, float e1, float e2, float e3,std::string angle_type){
        this->base_marker.p = {x,y,z};
        this->base_marker.M = KDL::Rotation::EulerZYZ(e1,e2,e3);
}

int ComauSmartSix::fk(float* q_in,float& x,float& y,float& z,float& e1, float& e2, float& e3){
        KDL::Frame pose;

        int n = this->tree.getNrOfJoints();
        KDL::JntArray j_q_in = JntArray(n);
        for(int i = 0; i < n; i++)
                j_q_in(i)=q_in[i];


        this->chain_solver_fk->JntToCart(j_q_in,pose);

        pose = pose * this->tool;

        x = pose.p[0];
        y = pose.p[1];
        z = pose.p[2];

        double roll,pitch,yaw;
        pose.M.GetRPY(roll,pitch,yaw);

        e1 = roll;
        e2 = pitch;
        e3 = yaw;

}

int ComauSmartSix::fk(float* q_in,float& x,float& y,float& z,float& qx, float& qy, float& qz,float& qw){
        KDL::Frame pose;

        int n = this->tree.getNrOfJoints();
        KDL::JntArray j_q_in = JntArray(n);
        for(int i = 0; i < n; i++)
                j_q_in(i)=q_in[i];

        this->chain_solver_fk->JntToCart(j_q_in,pose);

        pose = pose * this->tool;

        x = pose.p[0];
        y = pose.p[1];
        z = pose.p[2];

        double dqx,dqy,dqz,dqw;
        pose.M.GetQuaternion(dqx,dqy,dqz,dqw);
        qx = dqx;
        qy = dqy;
        qz = dqz;
        qw = dqw;

}

int ComauSmartSix::ik(float x, float y,float z,float roll, float pitch,float yaw,float* q_in,float* q_out,bool use_radians ){

        //TOOL
        /*KDL::Frame tool;
           tool.p = {0,0,-0.095f};
           tool.M = KDL::Rotation::EulerZYZ(0 , PI, 0);*/

        //KDL::Frame tool;
        //  tool.p = {-0.045f,0.0725f,-0.253f};
        //  tool.M = KDL::Rotation::EulerZYZ(0 , PI, 0);


        KDL::Frame cartesian;
        cartesian.p = {
                x/1000.0f,
                y/1000.0f,
                z/1000.0f
        };

        if(!use_radians) {
                roll = roll*PI/180.0f;
                pitch = pitch*PI/180.0f;
                yaw = yaw*PI/180.0f;
        }

        cartesian.M = KDL::Rotation::RPY(roll, pitch, yaw);


        cartesian = cartesian * this->tool.Inverse();

        unsigned int n = this->tree.getNrOfJoints();

        KDL::JntArray j_q_in = JntArray(n);
        KDL::JntArray j_q_out = JntArray(n);

        for(int i = 0; i < n; i++)
                j_q_in(i)=q_in[i];

        int c = chain_solver_ik_jl->CartToJnt( j_q_in,cartesian,j_q_out );


        for(int i = 0; i < n; i++)
                q_out[i] = j_q_out(i);


        return c;
}


int ComauSmartSix::ik(float x, float y,float z,float qx, float qy,float qz, float qw, float* q_in,float* q_out){

        KDL::Rotation rot = KDL::Rotation::Quaternion(qx,qy,qz,qw);
        double roll,pitch,yaw;
        rot.GetRPY(roll,pitch,yaw);
        return this->ik(x,y,z,roll,pitch,yaw,q_in,q_out,true);
}


}
