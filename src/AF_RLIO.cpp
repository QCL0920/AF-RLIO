#include "IMU_Processing.hpp"

#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/ndt.h>
#include <pcl_ros/features/normal_3d.h>

double normalized =1;

typedef Matrix<double, 1, 12> Matrix1x12;
typedef Matrix<double, 12, 12> Matrix12x12;

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];

double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false;
/**************************/

float res_last[100000] = {0.0};   float DET_RANGE = 300.0f;         const float MOV_THRESHOLD = 1.5f; 
mutex mtx_buffer;              condition_variable sig_buffer; 
string root_dir = ROOT_DIR;                 string map_file_path, lid_topic, imu_topic; static int num = 1;

double res_mean_last = 0.05, total_residual = 0.0;                                                 double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;                                        double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;                       double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0; 
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double radar_end_time = 0 , gps_end_time = 0 , Two_frame_time = 0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool point_selected_surf[100000] = {0}; bool lidar_pushed, flg_reset, flg_exit , radar_pushed = false, flg_EKF_inited;
bool odom_pushed = false;
bool gps_pushed = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>> pointSearchInd_surf;      
vector<BoxPointType> cub_needrm;              
vector<PointVector> Nearest_Points;           
vector<double> extrinT(3, 0.0);               
vector<double> extrinR(9, 0.0);               
deque<double> time_buffer;                    
deque<PointCloudXYZI::Ptr> lidar_buffer;      
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; 
deque<sensor_msgs::Imu::ConstPtr> imu_radar_buffer; 
deque<sensor_msgs::Imu::ConstPtr> imu_gps_buffer; 
deque<sensor_msgs::PointCloud2> cloudQueue;
deque<nav_msgs::Odometry> radar_odom;
deque<double> odom_time_buffer;
deque<pcl::PointCloud<RadarType>::Ptr> radar_buffer;
deque<double> radar_time_buffer;
pcl::PointCloud<RadarType>::Ptr radar_StaticCloud(new pcl::PointCloud<RadarType>());
pcl::PointCloud<RadarType>::Ptr radar_dynamicCloud(new pcl::PointCloud<RadarType>());


PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());           
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());        
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());        
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());       
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));       
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1)); 
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1)); 
PointCloudXYZI::Ptr _featsArray;
pcl::PointCloud<RadarType>::Ptr radarCloud(new pcl::PointCloud<RadarType>());
PointCloudXYZI::Ptr staticCloud(new PointCloudXYZI());
PointCloudXYZI::Ptr lidar_for_normal(new PointCloudXYZI());
PointCloudXYZI::Ptr endCloud(new PointCloudXYZI());
PointCloudXYZI::Ptr dynamicCloud(new PointCloudXYZI());PointCloudXYZI::Ptr StaticCloud(new PointCloudXYZI());


pcl::VoxelGrid<PointType> downSizeFilterSurf; pcl::VoxelGrid<PointType> downSizeFilterMap;  pcl::VoxelGrid<PointType> downSizeFilterICP;

KD_TREE ikdtree; 
bool islidar = true;
double lidarcov;
double lidarodomcov;
double radarodomcov;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);  
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0); 
V3D euler_cur;                                
V3D position_last(Zero3d);                    
V3D Lidar_T_wrt_IMU(Zero3d);                  
M3D Lidar_R_wrt_IMU(Eye3d);                   
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; state_ikfom state_point;                         
state_ikfom radar_state_point;   
Eigen::VectorXd eigenvalues;

vect3 pos_lid;                                   
nav_msgs::Path path;                      
nav_msgs::Odometry odomAftMapped;        
geometry_msgs::Quaternion geoQuat;        
geometry_msgs::PoseStamped msg_body_pose; 
static int num_radar = 0;
static int accuracy_lidar = 0;
static int accuracy_radar = 0;

float transformTobeMapped[6]; 


float radartransform[6];
float LastRadartransform[6];
Eigen::Vector3f euler_angles;

double lidar_initial_value = 1e-6;
double lidar_final_value = 1;
int lidar_total_iterations = 30;

double radar_initial_value = 1e-6;
double radar_final_value = 1;
int radar_total_iterations = 30;

nav_msgs::Path globalPath;

shared_ptr<Preprocess> p_pre(new Preprocess()); shared_ptr<ImuProcess> p_imu(new ImuProcess()); 
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;  

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); 
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

std::mutex mtx;

float surroundingkeyframeAddingDistThreshold;  float surroundingkeyframeAddingAngleThreshold; float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer; bool    recontructKdTree = false;
int updateKdtreeCount = 0 ; 


vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values initialEstimate_radar;
gtsam::Values optimizedEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;
float mappingSurfLeafSize;
int numberOfCores = 4;

/*gps参数*/
std::deque<nav_msgs::Odometry> gpsQueue;
std::deque<nav_msgs::Odometry> gpsQueue1;
std::deque<geometry_msgs::TwistWithCovarianceStamped> radar_twist;
double timeLaserInfoCur;
bool useGpsElevation;   
nav_msgs::Odometry thisGPS;
double gpsCov_x = 0;
double gpsCov_y = 0;
double gpsCov_z = 0;
Eigen::VectorXd lastoptpose(3);
Eigen::VectorXd errorgpspose(3);
Eigen::VectorXd gps_vel(3);
Eigen::MatrixXd gpsposeCovariance(6, 6);
deque<double> gps_time_buffer;

double alpha = 0.7; double previous_smoothed_p = 0.0; int frame_count = 0;


/*imu预积分参数*/
double imuGravity = 9.80511;
double imuAccNoise = 0.01;
double imuGyrNoise = 0.001;
gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);




void getCurPose(state_ikfom cur_state)
{   
        
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2,1,0);               
    transformTobeMapped[0] = eulerAngle(2);                    
    transformTobeMapped[1] = eulerAngle(1);                    
    transformTobeMapped[2] = eulerAngle(0);                    
    transformTobeMapped[3] = cur_state.pos(0);              
    transformTobeMapped[4] = cur_state.pos(1);              
    transformTobeMapped[5] = cur_state.pos(2);          
    }

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all(); }

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                                
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));        
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                     
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));        
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                     
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));           
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));           
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);     
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}
void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);     for (int i = 0; i < points_history.size(); i++)
        _featsArray->push_back(points_history[i]); }

BoxPointType LocalMap_Points;      bool Localmap_Initialized = false; void lasermap_fov_segment()
{
    cub_needrm.clear();     kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
        pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
        V3D pos_LiD = pos_lid;
        if (!Localmap_Initialized)
    {         for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
        float dist_to_map_edge[3][2];
    bool need_move = false;
        for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
                if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
        if (!need_move)
        return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
                if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);         }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
        if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();    
    scan_count++;
    double preprocess_start_time = omp_get_wtime();     
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                                           
    lidar_buffer.push_back(ptr);                                        
    time_buffer.push_back(msg->header.stamp.toSec());                   
    last_timestamp_lidar = msg->header.stamp.toSec();                   
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;    
     mtx_buffer.unlock();
    sig_buffer.notify_all(); 
    }

double timediff_lidar_wrt_imu = 0.0; bool timediff_set_flg = false;       
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
        if (!time_sync_en && abs(last_timestamp_imu - lidar_end_time) > 10.0)
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar scan end time: %lf", last_timestamp_imu, lidar_end_time);
    }
        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;         timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all(); 
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) 
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec()); 
    }

    double timestamp = msg->header.stamp.toSec(); 

    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
        imu_radar_buffer.clear();
        imu_gps_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    imu_radar_buffer.push_back(msg);
    imu_gps_buffer.push_back(msg);
    mtx_buffer.unlock();     
    sig_buffer.notify_all(); 
}


void radar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();     cloudQueue.push_back(*msg);
    sensor_msgs::PointCloud2 currentCloudMsg;
    currentCloudMsg = std::move(cloudQueue.front());
    cloudQueue.pop_front();
    pcl::moveFromROSMsg(currentCloudMsg, *radarCloud);
    radar_buffer.push_back(radarCloud);
    radar_time_buffer.push_back(msg->header.stamp.toSec());
    mtx_buffer.unlock();

}

void gps_cbk(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        mtx_buffer.lock();
        gpsQueue.push_back(*gpsMsg);
        gpsQueue1.push_back(*gpsMsg);
        gps_time_buffer.push_back(gpsMsg->header.stamp.toSec());
        mtx_buffer.unlock();
    }

void twist_cbk(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twistMsg)
{
    radar_twist.push_back(*twistMsg);

}

bool sync_packages(MeasureGroup &meas)
{
    if (islidar == true)
    {
        if (lidar_buffer.empty() || imu_buffer.empty() || radar_buffer.empty())         
        {
            return false;
        }

    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();

        if (meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            return false;
        }

        meas.lidar_beg_time = time_buffer.front();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature /1000; 
        lidar_pushed = true;
    }

    if (!radar_pushed){
       
        meas.radar = radar_buffer.front();
        meas.radar_beg_time = radar_time_buffer.front();


        if ((lidar_end_time - meas.radar_beg_time) > 0.08)
        {
            radar_buffer.pop_front();
            radar_time_buffer.pop_front();
            return false;
        }

        radar_end_time = meas.radar_beg_time ; 
        Two_frame_time = meas.lidar_beg_time - meas.radar_beg_time;
        radar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    meas.imu_radar.clear();
    meas.imu_gps.clear();
    
    
        if ( !gps_time_buffer.empty())
    {
        meas.gps_beg_time = gps_time_buffer.front();
        if((lidar_end_time - meas.gps_beg_time) > 0.1)
        {
            gpsQueue.pop_front();
            gps_time_buffer.pop_front();
            return false;
        }
        gps_pushed = true;
        gps_time_buffer.pop_front();
    }

        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) 
    {
        imu_time = imu_buffer.front()->header.stamp.toSec(); 
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front()); 
       
        imu_buffer.pop_front();
    }

    double start = std::min(lidar_end_time, meas.radar_beg_time);
    double end = std::max(lidar_end_time, meas.radar_beg_time);
    while (!imu_radar_buffer.empty())
    {   
        imu_time = imu_radar_buffer.front()->header.stamp.toSec(); 
        if (imu_time > end)
            break;

        if (imu_time>start && imu_time<end)
        {
            meas.imu_radar.push_back(imu_radar_buffer.front());
        }
        imu_radar_buffer.pop_front();
    }

        double start_gps = std::min(lidar_end_time, meas.gps_beg_time);
    double end_gps = std::max(lidar_end_time, meas.gps_beg_time);

    while (!imu_gps_buffer.empty())
    {
        imu_time = imu_gps_buffer.front()->header.stamp.toSec();
        if (imu_time > end_gps)
            break;
         if (imu_time>start_gps && imu_time < end_gps)
        {
            meas.imu_gps.push_back(imu_gps_buffer.front());
        }
        imu_gps_buffer.pop_front();
    }
    
    meas.lidar_end_time = lidar_end_time;


    lidar_buffer.pop_front();     time_buffer.pop_front();  
    radar_buffer.pop_front();
    radar_time_buffer.pop_front();

    radar_pushed = false;
    lidar_pushed = false;         odom_pushed  = false;
    gps_pushed = false;

    timeLaserInfoCur = meas.lidar_beg_time;
    return true;
    }
    else{
        if(imu_buffer.empty() || radar_buffer.empty())
        {
            return false;
        }

            double imu_time = imu_buffer.front()->header.stamp.toSec();
        meas.imu.clear();
        meas.imu_radar.clear();
        meas.imu_gps.clear();

        meas.radar = radar_buffer.front();
        meas.radar_beg_time = radar_time_buffer.front();
        lidar_end_time = radar_time_buffer.front();
        meas.radar_map->clear();

                for(int i=0;i<meas.radar->size();i++)
            {  
                if (meas.radar->points[i].sd_flg ==0)
                continue;
                pcl::PointXYZINormal radar_staticCloud;
                radar_staticCloud.x = meas.radar->points[i].x;
                radar_staticCloud.y = meas.radar->points[i].y;
                radar_staticCloud.z = meas.radar->points[i].z;
                radar_staticCloud.intensity = meas.radar->points[i].intensity;
                radar_staticCloud.normal_x = 0;
                radar_staticCloud.normal_y = 0;
                radar_staticCloud.normal_z = 0;
                meas.radar_map->push_back(radar_staticCloud);
            }


        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))         
        {
        imu_time = imu_buffer.front()->header.stamp.toSec();         
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());        
        imu_buffer.pop_front();
        }

            if (!gps_pushed && !gps_time_buffer.empty())
    {
        meas.gps_beg_time = gps_time_buffer.front();
        if((lidar_end_time - meas.gps_beg_time) > 0.07)
        {
            gpsQueue.pop_front();
            gps_time_buffer.pop_front();
            return false;
        }
        gps_pushed = true;
    }

    double start_gps = std::min(lidar_end_time, meas.gps_beg_time);
    double end_gps = std::max(lidar_end_time, meas.gps_beg_time);

    while (!imu_gps_buffer.empty())
    {
        imu_time = imu_gps_buffer.front()->header.stamp.toSec();
        if (imu_time > end_gps)
            break;
         if (imu_time>start_gps && imu_time < end_gps)
        {
            meas.imu_gps.push_back(imu_gps_buffer.front());
        }
        imu_gps_buffer.pop_front();
    }
    
    meas.lidar_end_time = lidar_end_time;

    if (!lidar_buffer.empty())
    {
        meas.lidar = lidar_buffer.front();
    lidar_buffer.pop_front();     time_buffer.pop_front();     
    }

    if (!radar_buffer.empty()){
    radar_buffer.pop_front();
    radar_time_buffer.pop_front();
    }

        
    if (!gps_time_buffer.empty())
    {
    gps_time_buffer.pop_front();
    }

    radar_pushed = false;
    lidar_pushed = false;             
    gps_pushed = false;

    timeLaserInfoCur = meas.radar_beg_time;
    return true;



    }
    

}

int process_increments = 0;
void map_incremental() {
    PointVector PointToAdd;                             
    PointVector PointNoNeedDownsample;                  
    PointToAdd.reserve(feats_down_size);                
    PointNoNeedDownsample.reserve(feats_down_size);         
    for (int i = 0; i < feats_down_size; i++)
    {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];             
            bool need_add = true;                                           
            BoxPointType Box_of_Point;                                      
            PointType downsample_result, mid_point;                                                 
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
                        for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)                     break;
                                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);        
                }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);         
            }
    }

    double st_time = omp_get_wtime();                                      
    add_point_size = ikdtree.Add_Points(PointToAdd, true);                 
    ikdtree.Add_Points(PointNoNeedDownsample, false);                      
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();     
    kdtree_incremental_time = omp_get_wtime() - st_time;               
    }

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1)); PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());         
void publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    if (scan_pub_en)     {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);         
        int size = laserCloudFullRes->points.size();                                                     
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1)); 
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudmsg);
                laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "body";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i],
                                &laserCloudWorld->points[i]);         
                                }
        *pcl_wait_save += *laserCloudWorld;     
        }
}
void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1)); 
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);     
                               }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void publish_frame_lidar(const ros::Publisher &pubLaserCloudFull_lidar)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);     laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    pubLaserCloudFull_lidar.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void publish_effect_world(const ros::Publisher &pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));     for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}
void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}
template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);     out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;     out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}
void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);     set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));     
}
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();    
    laserCloudOri->clear();                   
    corr_normvect->clear();                   
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];           
        PointType &point_world = feats_down_world->points[i]; 
        /* transform to world frame */
                V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);         
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)         
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])             
        continue;

        VF(4)
        pabcd;                                  
        point_selected_surf[i] = false; 
                if (esti_plane(pabcd, points_near, 0.1f))         
                {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);             
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());                                                   
            if (s > 0.9)             
            {
                point_selected_surf[i] = true;                   
                normvec->points[i].x = pabcd(0);                 
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;                 
                res_last[i] = abs(pd2);                         
            }
        }
    }

    effct_feat_num = 0; 
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {     
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];                         
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];             
            effct_feat_num++;                      
        }
    }

    res_mean_last = total_residual / effct_feat_num;     
    match_time += omp_get_wtime() - match_start;         
    double solve_start_ = omp_get_wtime();           
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);     
    ekfom_data.h.resize(effct_feat_num);                     
    Matrix1x12 accum_h_x = Matrix1x12::Zero();     
    Matrix12x12 accum_h_x_mat = Matrix12x12::Zero(); 
    Eigen::Vector3d D = Eigen::Vector3d::Zero();

        for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;                 
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;         
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this); 
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);                       
        V3D A(point_crossmat * C);                                         
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);         
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        
        accum_h_x = ekfom_data.h_x.block<1, 12>(i, 0);
        accum_h_x_mat += accum_h_x.transpose()*accum_h_x;

        ekfom_data.h(i) = -norm_p.intensity;     
        }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(accum_h_x_mat);
    eigenvalues = eigensolver.eigenvalues();

    solve_time += omp_get_wtime() - solve_start_; }

double logarithmicInterpolation(double initial_value, double final_value, int current_iteration, int total_iterations) 
{
    return initial_value * std::exp((static_cast<double>(current_iteration) / total_iterations) * std::log(final_value / initial_value));
}

double reverseLogarithmicInterpolation(double initial_value, double final_value, int current_iteration, int total_iterations) {
    return final_value * std::exp((1.0 - static_cast<double>(current_iteration) / total_iterations) * std::log(initial_value / final_value));
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}


Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

bool saveFrame()
{   
    if (cloudKeyPoses3D->points.empty())
        return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
                    
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw); 
        if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;

    
    return true;
}

void addOdomFactor()
{   
    
    if (cloudKeyPoses3D->points.empty())
    {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-6, 1e-6, 1e-6).finished());         
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        lidarodomcov = 1e-6;
    }
    else
    {   
        if(islidar == true&& accuracy_lidar > 0){
            accuracy_lidar --;
            lidarodomcov = reverseLogarithmicInterpolation(lidar_initial_value, lidar_final_value, accuracy_lidar, lidar_total_iterations);
            
        }
        else if (islidar == false&& accuracy_lidar < 1)          
        {   
            accuracy_lidar ++ ;
            lidarodomcov = logarithmicInterpolation(lidar_initial_value, lidar_final_value, accuracy_lidar, lidar_total_iterations);
        }
                   
            lidarodomcov = 1e-4;
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << lidarodomcov, lidarodomcov, lidarodomcov, lidarodomcov*100, lidarodomcov*100, lidarodomcov*100).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); 
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                                   
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        
            Eigen::Vector3d error_dx (lidarodomcov,0,0);
            double norm = error_dx.norm();

            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void addLoopFactor(){
    if (loopIndexQueue.empty())         
    return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;         
        int indexTo = loopIndexQueue[i].second;                  
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q ;                Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll ;
    q.normalize();
    return q ;
}

void updatePath(const PointTypePose &pose_in)
{
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x =  pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z =  pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I  );           
    T_b_lidar.pretranslate(state_point.offset_T_L_I);        

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;              
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;           
    Eigen::Isometry3d transCur = T_w_lidar;        

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}
double smooth_p(double current_p)
{
    double smoothed_p = alpha * current_p + (1 - alpha) * previous_smoothed_p;
    previous_smoothed_p = smoothed_p;
    return smoothed_p;
}

void  gpsEstimate(double& gpsCov_x, double& gpsCov_y, double& gpsCov_z)
{
    geometry_msgs::TwistWithCovarianceStamped radar_vel;
    nav_msgs::Odometry thisodometry;

    Eigen::VectorXd curoptpose(3);
    curoptpose[0] = transformTobeMapped[3];
    curoptpose[1] = transformTobeMapped[4];
    curoptpose[2] = transformTobeMapped[5];

    Eigen::VectorXd erroroptpose = curoptpose - lastoptpose;

    Eigen::VectorXd error_opt_gps(6);

    while(!radar_twist.empty())
    {
        radar_vel = radar_twist.front();
        radar_twist.pop_front();
    }

    error_opt_gps[0] = erroroptpose[0] - errorgpspose[0];
    error_opt_gps[1] = erroroptpose[1] - errorgpspose[1];
    error_opt_gps[2] = erroroptpose[2] - errorgpspose[2];
    error_opt_gps[3] = radar_vel.twist.twist.linear.x - gps_vel[0];
    error_opt_gps[4] = radar_vel.twist.twist.linear.y - gps_vel[1];
    error_opt_gps[5] = radar_vel.twist.twist.linear.z - gps_vel[2];

    gpsposeCovariance(0,0) = thisGPS.pose.covariance[0];
    gpsposeCovariance(1,1) = thisGPS.pose.covariance[4];
    gpsposeCovariance(2,2) = thisGPS.pose.covariance[8];
    gpsposeCovariance(3,3) = thisGPS.twist.covariance[0];
    gpsposeCovariance(4,4) = thisGPS.twist.covariance[7];
    gpsposeCovariance(5,5) = thisGPS.twist.covariance[14];

    double p = error_opt_gps.transpose() * gpsposeCovariance * error_opt_gps;

        if(frame_count < 5){
        gpsCov_x = gpsCov_y = gpsCov_z =1;
        return;
    }
    double smoothed_p = smooth_p(pow(p,2));
    Eigen::Vector3d error_dx (smoothed_p,0,0);
    double norm = error_dx.norm();
    gpsCov_x = gpsCov_y = gpsCov_z = smoothed_p;
}



void addGPSFactor()
{   
    if (gpsQueue.empty())
            return;
    else if(!cloudKeyPoses3D->points.empty())
    {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }        
    static PointType lastGPSPoint;
    while (!gpsQueue.empty() )
        {   

            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.1)
            {   
                gpsQueue.pop_front();
               
                gpsQueue1.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                break;
            }
            else
            {
                thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                float noise_x = thisGPS.pose.covariance[0];                         
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];      
                Eigen::Vector4d point(thisGPS.pose.pose.position.x, thisGPS.pose.pose.position.y, thisGPS.pose.pose.position.z, 1.0);
                Eigen::Matrix4d gps_start_end = p_imu->get_gps_pose_transform();
                Eigen::Vector4d transformed_point = gps_start_end * point;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;

                if (!useGpsElevation)                           
                {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
                }

                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;

                if (pointDistance(curGPSPoint, lastGPSPoint) < 0.5){
                    continue;
                }
                    
                else
                    lastGPSPoint = curGPSPoint;

                Eigen::VectorXd curgpspose(3);
                curgpspose[0] =thisGPS.pose.pose.position.x;
                curgpspose[1] =thisGPS.pose.pose.position.y;
                curgpspose[2] =thisGPS.pose.pose.position.z;

                static  Eigen::VectorXd lastgpspose(3);
                errorgpspose = curgpspose - lastgpspose;
                lastgpspose = curgpspose;

                gps_vel[0] = -thisGPS.twist.twist.linear.x;
                gps_vel[1] = thisGPS.twist.twist.linear.y;
                gps_vel[2] = thisGPS.twist.twist.linear.z;
                gtsam::Vector Vector3(3);

                gpsEstimate(gpsCov_x,gpsCov_y,gpsCov_z);

                Vector3 << gpsCov_x, gpsCov_y, gpsCov_z;
                                                
                gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);

                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);

                gtSAMgraph.add(gps_factor);
                break;
            }
            
        }
}

void saveKeyFramesAndFactor()
{   
    if (saveFrame() == false){
    return;
    }

    addOdomFactor();

    frame_count++;

    addGPSFactor();

    isam->update(gtSAMgraph, initialEstimate);

    isam->update();

    gtSAMgraph.resize(0);

    initialEstimate.clear();


    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateBestEstimate();

    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();

    thisPose3D.intensity = cloudKeyPoses3D->size();     
    cloudKeyPoses3D->push_back(thisPose3D);             
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);

    lastoptpose[0] = thisPose3D.x;
    lastoptpose[1] = thisPose3D.y;
    lastoptpose[2] = thisPose3D.z;
 

    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    state_ikfom state_updated = kf.get_x();     
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    state_updated.pos = pos;
    state_updated.rot =  q;
    state_point = state_updated;         
    kf.change_x(state_updated);      
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame);    
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    updatePath(thisPose6D);     
}

void recontructIKdTree(){
    if(recontructKdTree  &&  updateKdtreeCount >  0){
        /*** if path is too large, the rvis will crash ***/
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

                std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        mtx.lock();
        kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);                     pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);         
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);                         for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
            int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
            *subMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);        
        }
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                           
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);         
        downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

        ikdtree.reconstruct(subMapKeyFramesDS->points);
        updateKdtreeCount = 0;
        ROS_INFO("Reconstructed  ikdtree ");
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
    }
        updateKdtreeCount ++ ; 
}
void correctPoses()
{   

    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
                globalPath.poses.clear();
                int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

                        updatePath(cloudKeyPoses6D->points[i]);
        }
                recontructIKdTree();
        ROS_INFO("ISMA2 Update");
        aLoopIsClosed = false;
    }
}

void quaternionToEulerAngles(double x, double y, double z, double w,Eigen::Vector3f & euler_angles) {
        Eigen::Quaterniond q(w, x, y, z);

       euler_angles =  (q.toRotationMatrix().eulerAngles(2, 1, 0)).cast<float>();     
}

void publish_path_update(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);     
    string odometryFrame = "camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
            }
}

void removeDynamicPoints(const PointCloudXYZI::Ptr& laserCloud, const pcl::PointCloud<RadarType>::Ptr& radarCloud) {

    pcl::KdTreeFLANN<PointType> kdtree;

    kdtree.setInputCloud(laserCloud);

        std::vector<bool> isDynamic(laserCloud->size(), false);
    float threshold;

        for (const auto& radarPoint : *radarCloud) {
        if (radarPoint.recoveredSpeed <= -0.1) {
            threshold =2;         
            } 
            else 
            {
            threshold =1; 
        }

                std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        PointType searchPoint;
        searchPoint.x = radarPoint.x;
        searchPoint.y = radarPoint.y;
        searchPoint.z = radarPoint.z;
        kdtree.radiusSearch(searchPoint, threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        for (const auto& idx : pointIdxRadiusSearch) {
            isDynamic[idx] = true;
        }
    }

        for (size_t i = 0; i < laserCloud->size(); ++i) {
        if (!isDynamic[i]) {
            
            endCloud->push_back(laserCloud->points[i]);
        } else {
           
            dynamicCloud->push_back(laserCloud->points[i]);
        }
    }
}

void extractGroundPoints(const PointCloudXYZI::Ptr& inputCloud) {
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.01);

        seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    PointCloudXYZI::Ptr groundCloud(new PointCloudXYZI);
    extract.filter(*groundCloud);

        
    *endCloud += *groundCloud;

        groundCloud->clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);                   
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);                 
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);        
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);                          
    nh.param<string>("map_file_path", map_file_path, "");                           
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");                
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");                  
    nh.param<bool>("common/time_sync_en", time_sync_en, false);                     
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);            
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);                
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);                  
    nh.param<double>("cube_side_length", cube_len, 200);                            
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);                         
    nh.param<double>("mapping/fov_degree", fov_deg, 180);                           
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);                              
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);                              
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);                       
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);                      
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);                       
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);               
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);                      
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);                  
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);        
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);                   
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);                     
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());     
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); 
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    nh.param<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.1);
    nh.param<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
    nh.param<bool>("recontructKdTree", recontructKdTree, false);
    nh.param<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);
    nh.param<int>("numberOfCores", numberOfCores, 2);
    nh.param<float>("mappingSurfLeafSize", mappingSurfLeafSize, 1);
    
        nh.param<bool>("useGpsElevation", useGpsElevation, false);

    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);         
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);         
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);         
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());   
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);     Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);                 
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

        ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
        ros::Subscriber sub_imu   = nh.subscribe(imu_topic, 200000, imu_cbk);
        ros::Subscriber sub_radar = nh.subscribe("/filtered_points", 200000, radar_cbk);
        ros::Subscriber sub_GPS   = nh.subscribe<nav_msgs::Odometry> ("/odom", 200, gps_cbk);        
        ros::Subscriber sub_twist = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped> ("/radar_twist" , 200 , twist_cbk);
        ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
        ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
        ros::Publisher pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_lidar", 100000);
        ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
        ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
        ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
        ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
        ros::Publisher pubPathUpdate = nh.advertise<nav_msgs::Path>("af_rlio/path_update", 100000);                  
        signal(SIGINT, SigHandle);

        ros::Rate rate(5000);
        bool status = ros::ok();
        for (int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
        }
        while (status)
    {
                if (flg_exit)
            break;
                ros::spinOnce();
                if (sync_packages(Measures))
        {      
                        if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                Measures.imu.clear();                 continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();
                        p_imu->Process(Measures, kf, staticCloud);
            Eigen::Matrix4d T_start_end = p_imu->get_radar_pose_transform();
            
            pcl::transformPointCloud(*Measures.radar, *Measures.radar, T_start_end);


            radar_dynamicCloud->clear();
            radar_StaticCloud->clear();
            for (int i = 0; i< Measures.radar->points.size(); i++)
            {
                if (Measures.radar->points[i].sd_flg == 1)
                radar_StaticCloud->push_back(Measures.radar->points[i]);
                else
                radar_dynamicCloud->push_back(Measures.radar->points[i]);
            }

            state_point = kf.get_x();
            radar_state_point = state_point;

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      
            if (staticCloud->empty() || (staticCloud == NULL))             
            {
                first_lidar_time = Measures.lidar_beg_time;                 
                p_imu->first_lidar_time = first_lidar_time;                 
                continue;
            }

                                        
            endCloud-> clear();

            if (radar_dynamicCloud->size() !=0 && islidar ==true)
                        {
                removeDynamicPoints(staticCloud, radar_dynamicCloud);
                if (dynamicCloud->size()>10)
                {
                    extractGroundPoints(dynamicCloud);
                }
            }
            else
            {
                *endCloud = *staticCloud;
            }

            staticCloud->clear();
            
            feats_undistort->clear();
            feats_down_body->clear();
                                
            if (islidar ==true)
            {
            for (const auto& point : *endCloud) {
                if (point.x>2)
                feats_undistort->push_back(point);
                }
            }
            else{
                *feats_undistort = *endCloud;
            }
         
            lidar_for_normal->clear();
            StaticCloud->clear();

            *lidar_for_normal = *Measures.lidar;
            downSizeFilterSurf.setInputCloud(lidar_for_normal);             
            downSizeFilterSurf.filter(*lidar_for_normal);       

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;            
            
            lasermap_fov_segment();

            if (islidar==true)
            {
            downSizeFilterSurf.setInputCloud(feats_undistort);             
            downSizeFilterSurf.filter(*feats_down_body);       
            }
            else{
                *feats_down_body = *feats_undistort;

            }

            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); 
            pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
            pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> ne;
            ne.setInputCloud(lidar_for_normal);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(0.5);              
            ne.compute(*cloud_normals);

            for (size_t i = 0; i < lidar_for_normal->points.size(); ++i) 
            {
                lidar_for_normal->points[i].normal_x = cloud_normals->points[i].normal_x;
                lidar_for_normal->points[i].normal_y = cloud_normals->points[i].normal_y;
                lidar_for_normal->points[i].normal_z = cloud_normals->points[i].normal_z;
                lidar_for_normal->points[i].curvature = cloud_normals->points[i].curvature;
            }

            pcl::PointIndices::Ptr edge_indices(new pcl::PointIndices);
            pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);

            float edge_curvature_threshold = 0.08;              float plane_curvature_threshold = 0.01;

            for (size_t i = 0; i < lidar_for_normal->points.size(); ++i) 
            {
                float curvature = lidar_for_normal->points[i].curvature;
                if (curvature > edge_curvature_threshold ) 
                {
                    edge_indices->indices.push_back(i);
                } 
                else if (curvature < edge_curvature_threshold ) 
                {
                    plane_indices->indices.push_back(i);
                }
            }

            double w;
            w= (edge_indices->indices.size()*1.0)/(plane_indices->indices.size()*1.0);
            int cot;
            if (islidar == true){
                if (normalized < 0.1)
                {
                    islidar = false;
                    Measures.islidar = false;
                }
                else
                {
                    islidar = true;
                    Measures.islidar = true;
                }
            }
            else
            {
                if(w < 0.01){
                    cot = 0;
                }
                else{
                    cot++;
                }
                if (cot >= 10){
                    islidar = true;
                    Measures.islidar = true;
                }
                else{
                    islidar = false;
                    Measures.islidar = false;
                }
            }
            t1 = omp_get_wtime();                                          
            feats_down_size = feats_down_body->points.size();  

            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);                     
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));                     
                        }
                            ikdtree.Build(feats_down_world->points);                 
                }
                continue;
            }
                int featsFromMapNum = ikdtree.validnum();
                kdtree_size_st = ikdtree.size();

            
                feats_down_world->resize(feats_down_size);

                V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl; 


            pointSearchInd_surf.resize(feats_down_size);             
            Nearest_Points.resize(feats_down_size);                  
            int rematch_num = 0;
            bool nearest_search_en = true; 
            t2 = omp_get_wtime();
            
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(0.001, solve_H_time);
            auto dx_return = kf.get_dx();

            Eigen::Vector3d error_dx (dx_return(0,0), dx_return(1,0), dx_return(2,0));
            double norm = edge_indices->indices.size();
            state_point = kf.get_x();
            
            Eigen::VectorXd newMatrix = eigenvalues.segment(6, 3);
            
            double sum = newMatrix.sum();
            Eigen::VectorXd normalizedMatrix = newMatrix / sum;

            normalized = normalizedMatrix[0];

            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            getCurPose(state_point);

            quaternionToEulerAngles(Measures.odom->pose.pose.orientation.x, Measures.odom->pose.pose.orientation.y, Measures.odom->pose.pose.orientation.z, Measures.odom->pose.pose.orientation.w, euler_angles);

            saveKeyFramesAndFactor();

            correctPoses();

            num_radar++;

            publish_odometry(pubOdomAftMapped);

            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            publish_path(pubPath);
            publish_path_update(pubPathUpdate); 

            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
            {
                publish_frame_body(pubLaserCloudFull_body);
                publish_frame_lidar(pubLaserCloudFull_lidar);
            }
                                    dynamicCloud->clear();
            

            /*** Debug 参数 ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;                                         
                s_plot2[time_log_counter] = feats_undistort->points.size();                 
                s_plot3[time_log_counter] = kdtree_incremental_time;                        
                s_plot4[time_log_counter] = kdtree_search_time;                             
                s_plot5[time_log_counter] = kdtree_delete_counter;                          
                s_plot6[time_log_counter] = kdtree_delete_time;                             
                s_plot7[time_log_counter] = kdtree_size_st;                                 
                s_plot8[time_log_counter] = kdtree_size_end;                                
                s_plot9[time_log_counter] = aver_time_consu;                                
                s_plot10[time_log_counter] = add_point_size;                                
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    return 0;
}
