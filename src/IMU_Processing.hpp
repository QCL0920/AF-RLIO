#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Core>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <Eigen/Dense>
#include "use-ikfom.hpp"


#define MAX_INI_COUNT (20)


const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };


class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix4d get_radar_pose_transform();
  Eigen::Matrix4d get_gps_pose_transform();
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;      
  V3D cov_acc;             
  V3D cov_gyr;             
  V3D cov_acc_scale;      
  V3D cov_gyr_scale;       
  V3D cov_bias_gyr;        
  V3D cov_bias_acc;        
  double first_lidar_time; 
  Eigen::Matrix4d radar_start= Eigen::Matrix4d::Identity();;
  Eigen::Matrix4d radar_last= Eigen::Matrix4d::Identity();;

  Eigen::Matrix4d gps_start= Eigen::Matrix4d::Identity();;
  Eigen::Matrix4d gps_last= Eigen::Matrix4d::Identity();;

private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  void UndistortPcl_radar(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);

  PointCloudXYZI::Ptr cur_pcl_un_;        
  sensor_msgs::ImuConstPtr last_imu_;     
  deque<sensor_msgs::ImuConstPtr> v_imu_; 
  vector<Pose6D> IMUpose;                 
  vector<M3D> v_rot_pcl_;                 
  M3D Lidar_R_wrt_IMU;                    
  V3D Lidar_T_wrt_IMU;                    
  V3D mean_acc;                          
  V3D mean_gyr;                           
  V3D angvel_last;                        
  V3D acc_s_last;                         
  double start_timestamp_;                
  double last_lidar_end_time_;           
  int init_iter_num = 1;                  
  bool b_first_frame_ = true;            
  bool imu_need_init_ = true;             

};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                        
  Q = process_noise_cov();                    
  cov_acc = V3D(0.1, 0.1, 0.1);               
  cov_gyr = V3D(0.1, 0.1, 0.1);              
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001); 
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001); 
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;                    
  Lidar_T_wrt_IMU = Zero3d;                
  Lidar_R_wrt_IMU = Eye3d;                 
  last_imu_.reset(new sensor_msgs::Imu()); 
}

ImuProcess::~ImuProcess() {}

Eigen::Matrix4d ImuProcess::get_radar_pose_transform()
{
  Eigen::Matrix4d transform_radar = radar_start.inverse()*radar_last;
  return transform_radar;
}

Eigen::Matrix4d ImuProcess::get_gps_pose_transform()
{
  Eigen::Matrix4d transform_gps = gps_start.inverse()*gps_last;
  return transform_gps;
}

void ImuProcess::Reset()
{
    mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;                
  start_timestamp_ = -1;                
  init_iter_num = 1;                    
  v_imu_.clear();                         
  IMUpose.clear();                         
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}


void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}


void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}
void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}
void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. 初始化重力、陀螺偏差、acc和陀螺仪协方差
  /** 2. 将加速度测量值标准化为单位重力**/
  
  V3D cur_acc, cur_gyr;

  if (b_first_frame_)   {
    Reset();     N = 1;       b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;     const auto &gyr_acc = meas.imu.front()->angular_velocity;        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;                     mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;                     first_lidar_time = meas.lidar_beg_time;                        }
    for (const auto &imu : meas.imu)   {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
                        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) / (N - 1.0);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) / (N - 1.0);
                    N++;
  }
  state_ikfom init_state = kf_state.get_x();                    init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2); 
    init_state.bg = mean_gyr;                    init_state.offset_T_L_I = Lidar_T_wrt_IMU;   init_state.offset_R_L_I = Lidar_R_wrt_IMU;   kf_state.change_x(init_state);             
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();   init_P.setIdentity();                                                          init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;                          init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;                      init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;                     init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;                      init_P(21, 21) = init_P(22, 22) = 0.00001;                                     kf_state.change_P(init_P);                                                     last_imu_ = meas.imu.back();                                                 }

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  auto v_imu = meas.imu;                                              v_imu.push_front(last_imu_);                                        const double &imu_beg_time = v_imu.front()->header.stamp.toSec();   const double &imu_end_time = v_imu.back()->header.stamp.toSec();    const double &pcl_beg_time = meas.lidar_beg_time;                 
  /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);     
  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();   IMUpose.clear();                          
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;   M3D R_imu;                                          
  double dt = 0; 
  input_ikfom in; 
  int imu_radar_to_lidarlast = meas.imu.size() - meas.imu_radar.size();
  int imu_gps_to_lidarlast = meas.imu.size() - meas.imu_gps.size();

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);         auto &&tail = *(it_imu + 1);         if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);  
            acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); 
        if (head->header.stamp.toSec() < last_lidar_end_time_)
    {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
          }
    else
    {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

        in.acc = acc_avr;
    in.gyro = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

        kf_state.predict(dt, Q, in);


    /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();

            angvel_last = angvel_avr - imu_state.bg;                   acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);     for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];     }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;                                                                        IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix())); 

    int index = std::distance(v_imu.begin(), it_imu);
    if (index == imu_radar_to_lidarlast)
    {
      radar_start.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      radar_start.block<3, 1>(0, 3) = imu_state.pos;
    }

    if (index == imu_gps_to_lidarlast)
    {
      gps_start.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      gps_start.block<3, 1>(0, 3) = imu_state.pos;
    }

    if (index == std::distance(v_imu.begin(), v_imu.end()-2))
    {
      radar_last.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      radar_last.block<3, 1>(0, 3) = imu_state.pos;

      gps_last.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      gps_last.block<3, 1>(0, 3) = imu_state.pos;
    }

  }


      double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();          last_imu_ = meas.imu.back();           last_lidar_end_time_ = pcl_end_time; 
  /*** 在处理完所有的IMU预测后，剩下的就是对激光的去畸变了 ***/
    auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);         vel_imu << VEC_FROM_ARRAY(head->vel);        pos_imu << VEC_FROM_ARRAY(head->pos);        acc_imu << VEC_FROM_ARRAY(tail->acc);        angvel_avr << VEC_FROM_ARRAY(tail->gyr); 
        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)     {
      dt = it_pcl->curvature / double(1000) - head->offset_time; 
      /*变换到“结束”帧，仅使用旋转
       *注意：补偿方向与帧的移动方向相反
       *所以如果我们想补偿时间戳i到帧e的一个点
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei)  其中T_ei在全局框架中表示*/
      M3D R_i(R_imu * Exp(angvel_avr, dt)); 
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);                                         V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);                                                                                                       V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I); 
            it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

void ImuProcess::UndistortPcl_radar(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
{
  auto v_imu = meas.imu;                                              v_imu.push_front(last_imu_);                                        const double &imu_beg_time = v_imu.front()->header.stamp.toSec();   const double &imu_end_time = v_imu.back()->header.stamp.toSec();    const double &pcl_beg_time = meas.lidar_beg_time;                 
  /*** sort point clouds by offset time ***/
    const double &pcl_end_time = meas.lidar_end_time;     
  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();   IMUpose.clear();                          
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;   M3D R_imu;                                          
  double dt = 0; 
  input_ikfom in; 
  int imu_radar_to_lidarlast = meas.imu.size() - meas.imu_radar.size();
  int imu_gps_to_lidarlast = meas.imu.size() - meas.imu_gps.size();

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);         auto &&tail = *(it_imu + 1);         if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);  
            acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); 
        if (head->header.stamp.toSec() < last_lidar_end_time_)
    {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
          }
    else
    {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

        in.acc = acc_avr;
    in.gyro = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

        kf_state.predict(dt, Q, in);


    /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();

            angvel_last = angvel_avr - imu_state.bg;                   acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);     for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];     }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;                                                                        IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix())); 

    int index = std::distance(v_imu.begin(), it_imu);
    if (index == imu_radar_to_lidarlast)
    {
      radar_start.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      radar_start.block<3, 1>(0, 3) = imu_state.pos;
    }

    if (index == imu_gps_to_lidarlast)
    {
      gps_start.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      gps_start.block<3, 1>(0, 3) = imu_state.pos;
    }

    if (index == std::distance(v_imu.begin(), v_imu.end()-2))
    {
      radar_last.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      radar_last.block<3, 1>(0, 3) = imu_state.pos;

      gps_last.block<3, 3>(0, 0) = imu_state.rot.toRotationMatrix();
      gps_last.block<3, 1>(0, 3) = imu_state.pos;
    }

  }


      double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();          last_imu_ = meas.imu.back();           last_lidar_end_time_ = pcl_end_time; 
}

void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();

  if (meas.imu.empty())
  {
    return;
  };   ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
        IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);       imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
                  fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }
    if(meas.islidar == true)
  UndistortPcl(meas, kf_state, *cur_pcl_un_);
  if(meas.islidar == false)
  {
  UndistortPcl_radar(meas, kf_state);
  *cur_pcl_un_ = *meas.radar_map;
  }

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();

  }
