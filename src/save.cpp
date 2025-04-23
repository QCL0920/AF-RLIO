#include "ros/ros.h"
#include "bits/stdc++.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "GeographicLib/LocalCartesian.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Vector3Stamped.h>



void path_save(nav_msgs::Odometry odomAftMapped ){
 
	    //保存轨迹，path_save是文件目录,txt文件提前建好,/home/xxx/xxx.txt,
   			std::ofstream pose1("/home/qcl/bag/pose/B0-target33.txt", std::ios::app);
			pose1.setf(std::ios::scientific, std::ios::floatfield);
			pose1.precision(9);
	
			static double timeStart = odomAftMapped.header.stamp.toSec();
			auto T1 =ros::Time().fromSec(timeStart) ;
			pose1<< odomAftMapped.header.stamp -T1<< " "
              << -odomAftMapped.pose.pose.position.y << " "
              << odomAftMapped.pose.pose.position.z << " "
              << odomAftMapped.pose.pose.position.x << " "
              << odomAftMapped.pose.pose.orientation.x << " "
              << odomAftMapped.pose.pose.orientation.y << " "
              << odomAftMapped.pose.pose.orientation.z << " "
              << odomAftMapped.pose.pose.orientation.w << std::endl;
			pose1.close();
            
}
 
void path_saveend(nav_msgs::Path odomAftMapped ){
 
	    //保存轨迹，path_save是文件目录,txt文件提前建好,/home/xxx/xxx.txt,
   			std::ofstream pose1("/home/qcl/bag/pose/D0-rio-path15.txt", std::ios::app);
			pose1.setf(std::ios::scientific, std::ios::floatfield);
			pose1.precision(9);
	
			static double timeStart = odomAftMapped.header.stamp.toSec();
            geometry_msgs::PoseStamped poseend =odomAftMapped.poses.back();

            odomAftMapped.poses.pop_back();
			auto T1 =ros::Time().fromSec(timeStart) ;
			pose1<< odomAftMapped.header.stamp -T1<<" "
              << -poseend.pose.position.y<< " "
              << poseend.pose.position.z << " "
              << poseend.pose.position.x << " "
              << poseend.pose.orientation.w << " "
              << poseend.pose.orientation.y << " "
              << poseend.pose.orientation.z << " "
              << poseend.pose.orientation.w << std::endl;
			pose1.close();
            
}



//  void path_saveend(nav_msgs::Odometry odomAftMapped ){
 
// 	    //保存轨迹，path_save是文件目录,txt文件提前建好,/home/xxx/xxx.txt,
//    			std::ofstream pose1("/home/qcl/bag/pose/RURAL_E0-gt.txt", std::ios::app);
// 			pose1.setf(std::ios::scientific, std::ios::floatfield);
// 			pose1.precision(9);
	
// 			static double timeStart = odomAftMapped.header.stamp.toSec();
// 			auto T1 =ros::Time().fromSec(timeStart) ;
// 			pose1<< odomAftMapped.header.stamp -T1<< " "
//               << -odomAftMapped.pose.pose.position.y << " "
//               << odomAftMapped.pose.pose.position.z << " "
//               << odomAftMapped.pose.pose.position.x << " "
//               << odomAftMapped.pose.pose.orientation.x << " "
//               << odomAftMapped.pose.pose.orientation.y << " "
//               << odomAftMapped.pose.pose.orientation.z << " "
//               << odomAftMapped.pose.pose.orientation.w << std::endl;
// 			pose1.close();
            
// }
int main(int argc, char **argv){
    ros::init(argc, argv, "path_save");
    ros::NodeHandle nh;
    ros::Subscriber save_path = nh.subscribe<nav_msgs::Odometry>("/Odometry",100000, path_save);	    //保存轨迹，a_loam直接订阅话题/aft_mapped_to_init。
    // ros::Subscriber save_pathend = nh.subscribe<nav_msgs::Path>("fast_lio_sam/path_update",100, path_saveend);
    // ros::Subscriber save_pathend = nh.subscribe<nav_msgs::Odometry>("/odom1",10000, path_saveend);
    ros::spin();
     }