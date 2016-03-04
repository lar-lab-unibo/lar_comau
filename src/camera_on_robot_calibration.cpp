#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>


#include <tf/transform_listener.h>


#include <opencv2/opencv.hpp>

#include "geometry_msgs/Pose.h"

#include <tf/transform_broadcaster.h>

#include "lar_tools.h"

ros::Publisher pub;
//pcl::visualization::PCLVisualizer* viewer;

tf::Transform matToTF(cv::Mat& mat){
        tf::Vector3 origin;
        tf::Matrix3x3 tf3d;
        origin.setValue(
                static_cast<float>(mat.at<float>(0,3))/1000.0f,
                static_cast<float>(mat.at<float>(1,3))/1000.0f,
                static_cast<float>(mat.at<float>(2,3))/1000.0f
                );

        tf3d.setValue(
                static_cast<float>(mat.at<float>(0,0)), static_cast<float>(mat.at<float>(0,1)), static_cast<float>(mat.at<float>(0,2)),
                static_cast<float>(mat.at<float>(1,0)), static_cast<float>(mat.at<float>(1,1)), static_cast<float>(mat.at<float>(1,2)),
                static_cast<float>(mat.at<float>(2,0)), static_cast<float>(mat.at<float>(2,1)), static_cast<float>(mat.at<float>(2,2))
                );

        for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
                        tf3d[i][j] = mat.at<float>(i,j);

                }
                std::cout<<std::endl;

        }


        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        tfqt = tfqt.normalized();

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);
        return transform;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
        // Create a container for the data.
        /* sensor_msgs::PointCloud2 output;

           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>);
           pcl::PCLPointCloud2 pcl_pc;
           pcl_conversions::toPCL(*input, pcl_pc);
           pcl::fromPCLPointCloud2(pcl_pc, *cloud);



             sensor_msgs::PointCloud out;

           float dummy_query_data[10] = {
               0,0,-1,0,
               0,1,0,0,
               1,0,0,0,
               0,0,0,1};

           cv::Mat dummy_query = cv::Mat(2, 4, CV_32F, dummy_query_data);
            tf::Transform tf = matToTF(dummy_query);
               tf::TransformListener listener;

           pcl_ros::transformPointCloud ("lar_marker_111", *input, out,listener);
             //tf::TransformListener::("lar_marker_111",*input,out);
         */

        sensor_msgs::PointCloud2 out = *input;
        out.header.frame_id = "camera_link";
        pub.publish(out);
        //viewer->addPointCloud(cloud, "scene");

        //std::cout << "Received: "<<cloud->points.size()<<std::endl;
}

int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "my_pcl_tutorial");
        ros::NodeHandle nh;

        //viewer = new pcl::visualization::PCLVisualizer("viewer");

        // Create a ROS subscriber for the input point cloud
        //ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
        //pub = nh.advertise<sensor_msgs::PointCloud2> ("mycloud", 1);

        tf::TransformListener listener;

        tf::StampedTransform t_cam_marker;
        tf::StampedTransform t_0_marker;
        tf::StampedTransform t_0_6;

        tf::Transform t_6_0;
        tf::Transform t_marker_cam;

        tf::Transform t_6_cam;

        tf::TransformBroadcaster br;

        // Spin
        while(nh.ok()  ) {
                try{
                        listener.lookupTransform("camera_rgb_frame", "lar_marker_600", ros::Time(0), t_cam_marker);
                        listener.lookupTransform("base", "comau_t_0_base_marker",ros::Time(0), t_0_marker);
                        listener.lookupTransform("base", "comau_t06",ros::Time(0), t_0_6);

                        t_6_0 = t_0_6.inverse();
                        t_marker_cam = t_cam_marker.inverse();

                        t_6_cam = t_6_0 * t_0_marker;
                        t_6_cam = t_6_cam * t_marker_cam;

                        geometry_msgs::Pose pose;

                        lar_tools::geometrypose_to_tf(pose,t_6_cam,true);
                        tf::Matrix3x3 m(t_6_cam.getRotation());
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);

                        std::cout << pose.position.x<<","<<pose.position.y<<","<<pose.position.z<<" ; ";
                        std::cout << roll*180.0f/M_PI<<","<<pitch*180.0f/M_PI<<","<<yaw*180.0f/M_PI <<std::endl;

                        br.sendTransform(tf::StampedTransform(t_6_cam, ros::Time::now(), "comau_t06", "comau_t0CAM"));

                        

                }
                catch (tf::TransformException ex) {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();

                }
                ros::spinOnce();
                std::system("clear");
        }
}
