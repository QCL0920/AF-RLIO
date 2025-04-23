#include "ros/ros.h"
#include "bits/stdc++.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "GeographicLib/LocalCartesian.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>

#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <random>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

using namespace std;
GeographicLib::LocalCartesian geo;
double latitude = 36.390078599999995;
double longitude = 127.39906579999999;
double altitude = 74.459;

double x, y, z;

ros::Publisher pubOdom;
void callback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gpsvel)

{
    static bool gpsInited = true;

    if(gpsInited){
        geo.Reset(latitude, longitude, altitude);
        cout.precision(12);
        cout << fixed << gpsMsg->latitude << endl << gpsMsg->longitude << endl << gpsMsg->altitude << endl;
        gpsInited = false;
    }
    geo.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, x, y, z);
    nav_msgs::Odometry msgOdom;
    msgOdom.header.frame_id = "camera_init";
    msgOdom.header.stamp.fromSec(gpsMsg->header.stamp.toSec()-36.217);

    msgOdom.pose.pose.position.x = -x;
    msgOdom.pose.pose.position.y = -y;
    msgOdom.pose.pose.position.z = z;
    msgOdom.twist.twist.linear.x = gpsvel->twist.twist.linear.x;
    msgOdom.twist.twist.linear.y = gpsvel->twist.twist.linear.y;
    msgOdom.twist.twist.linear.z = gpsvel->twist.twist.linear.z;
    msgOdom.twist.covariance = gpsvel->twist.covariance;
    msgOdom.pose.covariance[0] = gpsMsg->position_covariance[0];
    msgOdom.pose.covariance[4] = gpsMsg->position_covariance[4];
    msgOdom.pose.covariance[8] = gpsMsg->position_covariance[8];

    pubOdom.publish(msgOdom);
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"odom_fusion");
    ros::NodeHandle nh;
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 2000);
    message_filters::Subscriber<sensor_msgs::NavSatFix> subGps(nh, "/ublox_gps/fix",1000 );
    message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> subvel(nh, "/ublox_gps/fix_velocity", 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subGps, subvel);
    sync.registerCallback(boost::bind(&callback,_1,_2));
    ros::spin();
}
