/**
 * Copyright(C) 2020, Isao Hara
 * All rights reserved
 *
 * License: The MIT License
 */

#ifndef CAM_SHM_MEM_H_
#define CAM_SHM_MEM_H_
#pragma once

#include <shm_image_save/shmem.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

namespace shm_image_save
{
class ImageSharedMem
{
public:
  ImageSharedMem(int shm_id=SHM_IMAGE_ID, int size=SHM_IMAGE_LEN);

  ~ImageSharedMem(){ }

  sensor_msgs::CameraInfoPtr get_cam_info();
  sensor_msgs::ImagePtr get_cam_image();
  sensor_msgs::PointCloud2Ptr get_pcl_data();

private:
  void map_cam_shm_data(int shm_id, int size);
  cam_shm_data *cam_data_;

  int shmid_;
  int shm_size_;

  sensor_msgs::CameraInfoPtr info_;
  sensor_msgs::ImagePtr img_;
  sensor_msgs::PointCloud2Ptr pcl_;
};

}

#endif
