#include "radar_ego_velocity_estimator.h"
#include "rio_utils/radar_point_cloud.h"
#include "utility_radar.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
using namespace std;
typedef pcl::PointXYZI PointT;

Eigen::Vector3f translation_vector( 1.579, 0.04,-1.076);  

struct RadarPointXYZIRT
{
    PCL_ADD_POINT4D
        union
    {   
        float data_c[4] ;
        struct
        {
            float alpha;
            float beta;
            float range;
            float doppler;
            float power;
            float recoveredSpeed;
            uint16_t dotFlags;
            uint16_t denoiseFlag;
            uint16_t historyFrameFlag;
            uint16_t dopplerCorrectionFlag;
        };
        
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN48;
POINT_CLOUD_REGISTER_POINT_STRUCT (RadarPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z)(float, alpha, alpha)(float, beta, beta)
    (float, range, range)(float, doppler, doppler)(float, power, power)(float, recoveredSpeed, recoveredSpeed)
    (std::uint16_t, dotFlags, dotFlags) (std::uint16_t, denoiseFlag, denoiseFlag)
    (std::uint16_t, historyFrameFlag, historyFrameFlag)(std::uint16_t, dopplerCorrectionFlag, dopplerCorrectionFlag) 
)

struct RadarPointXYZV
{
    PCL_ADD_POINT4D
    float doppler;
    float recoveredSpeed;
    float intensity;
    uint16_t sd_flg;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN28;
POINT_CLOUD_REGISTER_POINT_STRUCT (RadarPointXYZV,
    (float, x, x) (float, y, y) (float, z, z)
    (float, doppler, doppler)(float, recoveredSpeed, recoveredSpeed)(float, intensity, intensity)
    (std::uint16_t, sd_flg, sd_flg)
)


using RadarPoint = RadarPointXYZIRT;
using RadarType = RadarPointXYZV;

struct Cluster {
    std::vector<RadarType> points;
    double centroidDoppler;
    double minDoppler;
    double maxDoppler;

};

class Radarpreprocess : public ParamServer
{
    private:
        int deskewFlag;
        ros::Subscriber subRadarCloud;
        ros::Publisher pubExtractedCloud;
        ros::Publisher pubRaserCloudInfo;
        ros::Publisher points_pub;
        ros::Publisher pub_twist;
        std::deque<sensor_msgs::PointCloud2> cloudQueue;
        sensor_msgs::PointCloud2 currentCloudMsg;

        pcl::PointCloud<RadarPoint>::Ptr laserCloudIn;
        pcl::PointCloud<RadarType>::Ptr fullCloud;
        pcl::PointCloud<RadarType>::Ptr extractedCloud;
        pcl::PointCloud<RadarType>::Ptr staticRadar;
        pcl::PointCloud<RadarType>::Ptr mergedCloud;

        std_msgs::Header cloudHeader;
        sensor_msgs::PointCloud2 cloudInfo;

        std::vector<Cluster> clusters;
        std::vector<std::vector<RadarType>> clusters2;
        std::vector<Cluster> clusters1;
        std::vector<std::vector<RadarType>> clusters4;

        rio::RadarEgoVelocityEstimator estimator;


    public:
    Radarpreprocess():

deskewFlag(0){
        subRadarCloud  =  nh.subscribe<sensor_msgs::PointCloud2>("/oculii_radar/point_cloud",1,&Radarpreprocess::radarcloudHandler,this ,ros::TransportHints().tcpNoDelay());
        
        pubRaserCloudInfo = nh.advertise<sensor_msgs::PointCloud2> ("/radarpoints", 1);
        points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
        pub_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar_twist", 5);
        allocateMemory();
        resetParameters();
}

void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<RadarPoint>());
        extractedCloud.reset(new pcl::PointCloud<RadarType>());
        fullCloud.reset(new pcl::PointCloud<RadarType>());
        staticRadar.reset(new pcl::PointCloud<RadarType>());
        mergedCloud.reset(new pcl::PointCloud<RadarType>());
        resetParameters();
}

void resetParameters(){
        laserCloudIn->clear();
        extractedCloud->clear();
        fullCloud->clear();
        staticRadar->clear();
        mergedCloud->clear();   
}

~Radarpreprocess(){}

void radarcloudHandler(const sensor_msgs::PointCloud2ConstPtr& radarCloudMsg)
    {

        if (!cachePointCloud(radarCloudMsg))
            return;
        
        cloudTransformation();

        projectPointCloud();

        cloudExtraction1();

        radarVelocity(radarCloudMsg);

        publishClouds();

        publishStaticCloud();

        resetParameters();
}

void radarVelocity(const sensor_msgs::PointCloud2ConstPtr& radarCloudMsg)
    {
        RadarPointCloudType radarpoint_raw;
        PointT radarpoint_xyzi;
        pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
        
        pcl::PointCloud<PointT>::Ptr radarcloud_xyzi( new pcl::PointCloud<PointT> );

        radarcloud_xyzi->header.frame_id = baselinkFrame;
        radarcloud_xyzi->header.seq = radarCloudMsg->header.seq;
        radarcloud_xyzi->header.stamp = radarCloudMsg->header.stamp.toSec() * 1e6;

        for(int i = 0; i < staticRadar->size(); i++)
        {
            radarpoint_raw.x = staticRadar->points[i].x;
            radarpoint_raw.y = staticRadar->points[i].y;
            radarpoint_raw.z = staticRadar->points[i].z;
            radarpoint_raw.intensity = staticRadar->points[i].intensity;
            radarpoint_raw.doppler = staticRadar->points[i].doppler;
            
            radarpoint_xyzi.x = staticRadar->points[i].x;
            radarpoint_xyzi.y = staticRadar->points[i].y;
            radarpoint_xyzi.z = staticRadar->points[i].z;
            radarpoint_xyzi.intensity = staticRadar->points[i].intensity;

            radarcloud_raw->points.push_back(radarpoint_raw);
            radarcloud_xyzi->points.push_back(radarpoint_xyzi);
        }

        sensor_msgs::PointCloud2 pc2_raw_msg;
        pcl::toROSMsg(*radarcloud_raw, pc2_raw_msg);
        pc2_raw_msg.header.stamp = radarCloudMsg->header.stamp;
        pc2_raw_msg.header.frame_id = baselinkFrame;

        Eigen::Vector3d v_r, sigma_v_r;
        sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
        clock_t start_ms = clock();

        if (estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
        {   
            geometry_msgs::TwistWithCovarianceStamped twist;
            twist.header.stamp         = currentCloudMsg.header.stamp;
            twist.twist.twist.linear.x = v_r.x();
            twist.twist.twist.linear.y = v_r.y();
            twist.twist.twist.linear.z = v_r.z();

            twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
            twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
            twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

            pub_twist.publish(twist);
        }

    }

void projectPointCloud(){   

    int cloudSize = laserCloudIn->points.size();
    int j = 0;
    for (int i = 1; i< cloudSize; i++)
    {   
        RadarType thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.doppler = laserCloudIn->points[i].doppler;
        thisPoint.recoveredSpeed = laserCloudIn->points[i].recoveredSpeed;
        thisPoint.intensity = laserCloudIn->points[i].power;
        int denoiseFlag = laserCloudIn->points[i].denoiseFlag;


        if (laserCloudIn->points[i].dotFlags == 0)
            continue;
        if (laserCloudIn->points[i].z<-1)
            continue;

        if (denoiseFlag == 1)
        {
            thisPoint.sd_flg = 1;
            staticRadar->push_back(thisPoint);
        }

        else
        {   
            thisPoint.sd_flg = 0;
            fullCloud->push_back(thisPoint);
        }
    }
}

void cloudExtraction1(){
    double weightDistance = 0; 
    double weightDoppler = 1; 
    double threshold = 2.0;
    std::vector<RadarType> targets;
        for (const auto& point : *fullCloud) 
        {
            targets.push_back(point);

        }

    double epsilon = 0.6; 
    int minPts = 3; 
    
    clusters1 = dbscanClustering(targets, epsilon, minPts); 


    double consistencyThreshold = 1.0;

    sortClustersByDoppler(clusters1);

    forward_moving(clusters1);

    reverse_moving(clusters1);
    
}

void forward_moving(std::vector<Cluster> clusters1){
        
         for (size_t i = 0; i < clusters1.size(); ++i) { 


        if (clusters1[i].points.size()<10) 
        continue;

        if (clusters1[i].maxDoppler-clusters1[i].minDoppler > 20) 
        continue;

        if (clusters1[i].centroidDoppler > -3) 
        continue;
        Cluster cluster_correct;
        for (const auto& point :clusters1[i].points){                            
        if (std::abs(point.recoveredSpeed - clusters1[i].centroidDoppler) > 3) 
            continue;
        else{
            cluster_correct.points.push_back(point);
            cluster_correct.maxDoppler = clusters1[i].maxDoppler;
            cluster_correct.minDoppler = clusters1[i].minDoppler;
            cluster_correct.centroidDoppler = clusters1[i].centroidDoppler;
            }
        }
                
        RadarType thisPoint2;
        for (int i=0; i<cluster_correct.points.size();i++){
            thisPoint2.x = cluster_correct.points[i].x;
            thisPoint2.y = cluster_correct.points[i].y;
            thisPoint2.z = cluster_correct.points[i].z;
            thisPoint2.doppler = cluster_correct.points[i].doppler;
            thisPoint2.recoveredSpeed = cluster_correct.points[i].recoveredSpeed;
            thisPoint2.intensity = cluster_correct.points[i].intensity;
            thisPoint2.sd_flg = 0;
            extractedCloud->push_back(thisPoint2);

        }

         }
    }

void reverse_moving(std::vector<Cluster> Clusters1){
        for (size_t i = 0; i < Clusters1.size(); ++i) { 
        if (Clusters1[i].points.size()<70)
        continue;


        if (Clusters1[i].maxDoppler-Clusters1[i].minDoppler > 20) 
        continue;

        if (Clusters1[i].centroidDoppler <2)
        continue;

        Cluster cluster_correct;

        for (const auto& point :Clusters1[i].points){                            
        if (std::abs(point.recoveredSpeed - Clusters1[i].centroidDoppler) > 0.1) 
            continue;
        else{
            cluster_correct.points.push_back(point);
            cluster_correct.maxDoppler = Clusters1[i].maxDoppler;
            cluster_correct.minDoppler = Clusters1[i].minDoppler;
            cluster_correct.centroidDoppler = Clusters1[i].centroidDoppler;
            }
        }
                
        RadarType thisPoint2;
        for (int i=0; i<cluster_correct.points.size();i++){
            thisPoint2.x = cluster_correct.points[i].x;
            thisPoint2.y = cluster_correct.points[i].y;
            thisPoint2.z = cluster_correct.points[i].z;
            thisPoint2.doppler = cluster_correct.points[i].doppler;
            thisPoint2.recoveredSpeed = cluster_correct.points[i].recoveredSpeed;
            thisPoint2.intensity = cluster_correct.points[i].intensity;
            thisPoint2.sd_flg = 0;
            extractedCloud->push_back(thisPoint2);

        }
    }
}

void cloudExtraction(){

        int count = 0;
        int k = 2;
        std::vector<RadarType> targets;
        for (const auto& point : *fullCloud) 
        {
            targets.push_back(point);
        }
        int maxIterations = 100;
        clusters = kMeansClustering(targets, k, maxIterations);

        for (int i = 0; i < k; ++i) {
        }

            RadarType thisPoint;
            std::vector<RadarType> targets2;
            for(int j=0; j<1; j++){
            for(int i=0; i<clusters[j].points.size();++i){
            thisPoint.x = clusters[j].points[i].x;
            thisPoint.y = clusters[j].points[i].y;
            thisPoint.z = clusters[j].points[i].z;
            thisPoint.doppler = clusters[j].points[i].doppler;
            thisPoint.recoveredSpeed = clusters[j].points[i].recoveredSpeed;
            thisPoint.intensity = 0;

            targets2.push_back(thisPoint);
            }
        }

    double epsilon = 0.1; 
    int minPts = 3; 
    int consistencyThreshold =1;

     for (size_t i = 0; i < clusters2.size(); ++i) {
        
        if (clusters2[i].size()>30){
            if (checkIntensityConsistency(clusters2[i], consistencyThreshold)){
        
            RadarType thisPoint2;
            for(int num = 0; num < clusters2[i].size(); num++)
            {
            
            thisPoint2.x = clusters2[i][num].x;
            thisPoint2.y = clusters2[i][num].y;
            thisPoint2.z = clusters2[i][num].z;
            thisPoint2.doppler = clusters2[i][num].doppler;
            thisPoint2.recoveredSpeed = clusters2[i][num].recoveredSpeed;
            thisPoint2.intensity = 0;
            thisPoint2.sd_flg = 0;
            extractedCloud->push_back(thisPoint2);
                }
            }
        }
    }
}

bool compareClusterByDoppler(const Cluster& c1, const Cluster& c2) {
    double sumDoppler1 = 0.0, sumDoppler2 = 0.0;
    for (const auto& p : c1.points) {
        sumDoppler1 += p.doppler;
    }
    for (const auto& p : c2.points) {
        sumDoppler2 += p.doppler;
    }
    double avgDoppler1 = sumDoppler1 / c1.points.size();
    double avgDoppler2 = sumDoppler2 / c2.points.size();
    return avgDoppler1 < avgDoppler2;
}

bool checkIntensityConsistency2(const std::vector<RadarType>& cluster, double consistencyThreshold) {
    double meanIntensity = calculateMeanIntensity(cluster);

    if (meanIntensity>-8){
        return false;
    }

    else
    return true; 
}

bool checkIntensityConsistency(const std::vector<RadarType>& cluster, double consistencyThreshold) {
    double meanIntensity = calculateMeanIntensity(cluster);

    int i = 0;

    for (const auto& point : cluster) {
        if (std::abs(point.doppler - meanIntensity) >5) {
            i++; 
        }
    }
    
    if (i>(cluster.size())/20){
        return false;
    }
    else
    return true; 
}

double calculateMeanIntensity(const std::vector<RadarType>& cluster) {
    double sumIntensity = 0.0;
    for (const auto& point : cluster) {
        sumIntensity += point.doppler;
    }
    return sumIntensity / cluster.size();
}

std::vector<Cluster> dbscanClustering(const std::vector<RadarType>& points, double epsilon, int minPts) {
    std::vector<Cluster> clusters;
    std::vector<bool> visited(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (!visited[i]) {
            visited[i] = true;
            std::vector<size_t> neighbors = densityConnectedPoints(points, points[i], epsilon);
            if (neighbors.size() >= minPts) {
                Cluster cluster;
                
                cluster.points.push_back(points[i]);
                visited[i] = true; 
                double maxDoppler = points[i].recoveredSpeed;
                double minDoppler = points[i].recoveredSpeed;
                for (size_t j = 0; j < neighbors.size(); ++j) {
                    size_t index = neighbors[j];
                    if (!visited[index]) {
                        cluster.points.push_back(points[index]);
                        visited[index] = true;
                        std::vector<size_t> nextNeighbors = densityConnectedPoints(points, points[index], epsilon);
                        for (size_t k = 0; k < nextNeighbors.size(); ++k) {
                            if (std::find(neighbors.begin(), neighbors.end(), nextNeighbors[k]) == neighbors.end()) {
                                neighbors.push_back(nextNeighbors[k]);
                            }
                        }
                        if (points[index].recoveredSpeed > maxDoppler) {
                            maxDoppler = points[index].recoveredSpeed;
                        }

                        if (points[index].recoveredSpeed < minDoppler) {
                            minDoppler = points[index].recoveredSpeed;
                        }
                    }
                }
                cluster.maxDoppler = maxDoppler;
                cluster.minDoppler = minDoppler;
                                double sumDoppler = 0.0;
                for (const auto& point : cluster.points) {
                    sumDoppler += point.recoveredSpeed;
                }
                cluster.centroidDoppler = sumDoppler / cluster.points.size();

                clusters.push_back(cluster);
            }
        }
    }

    return clusters;
}

void sortClustersByDoppler(std::vector<Cluster>& clusters) {
    std::sort(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
        return a.centroidDoppler < b.centroidDoppler;
    });
}

bool isDensityReachable(const RadarType& point1, const RadarType& point2, double epsilon) {
    return distance(point1, point2) <= epsilon;
}

std::vector<size_t> densityConnectedPoints(const std::vector<RadarType>& points, const RadarType& point, double epsilon) {
    std::vector<size_t> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (isDensityReachable(points[i], point, epsilon)) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

std::vector<std::vector<RadarType>> dbscanClustering2(const std::vector<RadarType>& points, double epsilon, int minPts, double weightDistance, double weightDoppler) {
    std::vector<std::vector<RadarType>> clusters;
    std::vector<bool> visited(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (!visited[i]) {
            visited[i] = true;
            std::vector<RadarType> cluster;
            cluster.push_back(points[i]);

            std::vector<size_t> neighbors = findNeighbors2(points, i, epsilon, weightDistance, weightDoppler);
            if (neighbors.size() >= minPts) {
                for (size_t j = 0; j < neighbors.size(); ++j) {
                    size_t neighborIndex = neighbors[j];
                    if (!visited[neighborIndex]) {
                        visited[neighborIndex] = true;
                        std::vector<size_t> neighborNeighbors = findNeighbors2(points, neighborIndex, epsilon, weightDistance, weightDoppler);
                        if (neighborNeighbors.size() >= minPts) {
                            neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
                        }
                    }
                    bool isAlreadyInCluster = false;
                    for (const auto& point : cluster) {
                        if (distance2(points[neighborIndex], point, weightDistance, weightDoppler) <= epsilon) {
                            isAlreadyInCluster = true;
                            break;
                        }
                    }
                    if (!isAlreadyInCluster) {
                        cluster.push_back(points[neighborIndex]);
                    }
                }
            }
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

double dopplerDistance(const RadarType& p1, const RadarType& p2) {
    return std::abs(p1.doppler - p2.doppler);
}

std::vector<std::vector<RadarType>> dbscanClustering3(const std::vector<RadarType>& points, double epsilon, int minPts) {
    std::vector<std::vector<RadarType>> clusters;
    std::vector<bool> visited(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (!visited[i]) {
            visited[i] = true;
            std::vector<RadarType> cluster;
            cluster.push_back(points[i]);

            for (size_t j = 0; j < points.size(); ++j) {
                if (i != j && dopplerDistance(points[i], points[j]) <= epsilon) {
                    cluster.push_back(points[j]);
                    visited[j] = true;
                }
            }

            if (cluster.size() >= minPts) {
                clusters.push_back(cluster);
            }
        }
    }

    return clusters;
}

double distance(const RadarType& p1, const RadarType& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    }
    
double distance2(const RadarType& p1, const RadarType& p2, double weightDistance, double weightDoppler) {
    double distanceComponent = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    double dopplerComponent = std::abs(p1.doppler - p2.doppler);
    return weightDistance * distanceComponent + weightDoppler * dopplerComponent;
}

std::vector<size_t> findNeighbors(const std::vector<RadarType>& points, size_t pointIndex, double epsilon)
    {
    std::vector<size_t> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i != pointIndex && distance(points[i], points[pointIndex]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

std::vector<size_t> findNeighbors2(const std::vector<RadarType>& points, size_t pointIndex, double epsilon, double weightDistance, double weightDoppler) {
    std::vector<size_t> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i != pointIndex && distance2(points[i], points[pointIndex], weightDistance, weightDoppler) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

std::vector<size_t> findNeighbors3(const std::vector<RadarType>& points, size_t pointIndex, double epsilon) {
    std::vector<size_t> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i != pointIndex && std::abs(points[i].doppler - points[pointIndex].doppler) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

std::vector<Cluster> kMeansClustering(const std::vector<RadarType>& data, int k, int maxIterations) {
    std::vector<Cluster> clusters(k);
    
    for (int i = 0; i < k; ++i) {
        clusters[i].centroidDoppler = data[rand() % data.size()].doppler;
    }
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        for (auto& cluster : clusters) {
            cluster.points.clear();
        }
        
        for (const auto& point : data) {
            double minDist = std::numeric_limits<double>::max();
            int closestClusterIdx = 0;
            for (int i = 0; i < k; ++i) {
                double dist = fabs(point.doppler - clusters[i].centroidDoppler);
                if (dist < minDist) {
                    minDist = dist;
                    closestClusterIdx = i;
                }
            }
            clusters[closestClusterIdx].points.push_back(point);
        }

        for (auto& cluster : clusters) {
            if (!cluster.points.empty()) {
                double sumDoppler = 0.0;
                for (const auto& point : cluster.points) {
                    sumDoppler += point.doppler;
                }
                cluster.centroidDoppler = sumDoppler / cluster.points.size();
            }
        }
    }

    std::sort(clusters.begin(), clusters.end(), [](const Cluster& c1, const Cluster& c2) {
        return c1.centroidDoppler > c2.centroidDoppler;
    });
    return clusters;
}       

void transformPointCloud(pcl::PointCloud<RadarPoint>::Ptr cloud, const Eigen::Matrix3f& rotation_matrix, const Eigen::Vector3f& translation_vector) 
    {
        for (auto& point : *cloud) 
        {
            Eigen::Vector3f point_position(point.x, point.y, point.z);
            Eigen::Vector3f transformed_point = rotation_matrix * point_position + translation_vector;

            point.x = transformed_point.x();
            point.y = transformed_point.y();
            point.z = transformed_point.z();
        }
        
    }

void cloudTransformation()
    {

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << 0, 0, 1,
                       -0.9962, 0.0872, 0,
                       0.0872, -0.9962, 0;  //oculii

        for (auto& point : *laserCloudIn) 
        {
            Eigen::Vector3f point_position(point.x, point.y, point.z);
            Eigen::Vector3f transformed_point = rotation_matrix * point_position + translation_vector;

            point.x = transformed_point.x();
            point.y = transformed_point.y();
            point.z = transformed_point.z();
        }
    }
    
bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {   
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;
        if (1)
        {
            currentCloudMsg = std::move(cloudQueue.front());
            cloudQueue.pop_front();
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else
        {
            ros::shutdown();
        }

        cloudHeader = currentCloudMsg.header;
        double cloudSize = laserCloudIn->size();
        return true;

    }

void publishClouds()
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*extractedCloud, tempCloud);
        tempCloud.header.stamp = cloudHeader.stamp;
        tempCloud.header.frame_id = "body";
        

        pubRaserCloudInfo.publish(tempCloud);

    }

void publishStaticCloud()
    {
        sensor_msgs::PointCloud2 tempCloud;
        *mergedCloud = *extractedCloud + *staticRadar;
        pcl::toROSMsg(*mergedCloud, tempCloud);
        tempCloud.header.stamp = cloudHeader.stamp;
        tempCloud.header.frame_id = "body";
        points_pub.publish(tempCloud);
    }  
};

   

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");


    Radarpreprocess RP;
    
    ROS_INFO("\033[1;32m----> Radar Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
