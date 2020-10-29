/**
 * nodelet for saving image data to shared memory
 *  Copyright(C) 2020 Isao Hara
 *  License: Apache License v2.0
 */

#ifndef PLUGIN_SHM_SAVER_H_
#define PLUGIN_SHM_SAVER_H_
#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <shm_image_save/shmem.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#define CAMERA_INFO  "camera_info"
#define CAMERA_IMAGE "camera_image"
#define PCL_MESSAGE  "depth_registered/points"

#define SHM_IMAGE_ID (234567)
#define SHM_IMAGE_LEN (30000000)
#define MAX_MSGS (2)


namespace shm_image_save
{

typedef struct cam_shm_data
{
  u_int32_t cam_info_size;
  u_int32_t cam_image_size;
  u_int32_t pcl_size;

  u_int32_t cam_info_offset;
  u_int32_t cam_image_offset;
  u_int32_t pcl_offset;

  u_int32_t cam_info_count;
  u_int32_t cam_image_count;
  u_int32_t pcl_count;

  unsigned char data[SHM_IMAGE_LEN - 24]; // 4*6
} cam_shm_data;


/**
 * Class 
 */
class plugin_nodelet_shm_saver : public nodelet::Nodelet
{
public:
  void onInit();

private:
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg);
  void callbackRsCloud(const sensor_msgs::PointCloud2ConstPtr &msg);
  void callbackColorImage(const sensor_msgs::ImageConstPtr & image_msg);

  ros::NodeHandle nh_;

  ros::Subscriber cam_info_sub_;
  ros::Subscriber cam_image_sub_;
  ros::Subscriber rs_cloud_sub_;
  

  // Shared Memory
  int m_shmid_;
  cam_shm_data *m_cam_;
};


}
#endif
