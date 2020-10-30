/*
 *
 *
 * Copyright(C) 2016-2020 RT Corp.
 *  All rights reserved.
 *
 *  License: Apache License v2.0
 *
 */
#include <shm_image_save/cam_shm.h>

namespace shm_image_save
{

/**
 * @brief Constructor for VisionBase
 * @fn VisionBase::VisionBase()
 */
ImageSharedMem::ImageSharedMem(int shm_id, int size)
		: shmid_(0), cam_data_(NULL), shm_size_(size)
{
  map_cam_shm_data(shm_id, size);
}

void 
ImageSharedMem::map_cam_shm_data(int shm_id, int size)
{
  void *res;
  res = map_shared_mem(&shmid_, shm_id, size,0);
  if(res == NULL){
    res = map_shared_mem(&shmid_, shm_id, size,1);
    if (res < 0){
      fprintf(stderr, "Fail to map cam_shm_data\n");
    }
    memset(res, 0, size);
  }else{
    cam_data_ = (cam_shm_data *)res;
  }
  return;
}

sensor_msgs::CameraInfoPtr
ImageSharedMem::get_cam_info()
{
  uint32_t serial_size = cam_data_->cam_info_size;
  unsigned char *addr = cam_data_->data + cam_data_->cam_info_offset;
  sensor_msgs::CameraInfoPtr info_(new sensor_msgs::CameraInfo());
  ros::serialization::IStream stream((uint8_t *)addr, serial_size);
  ros::serialization::deserialize(stream, *info_);

  return info_;
}

/*
 *
 */
sensor_msgs::ImagePtr
ImageSharedMem::get_cam_image()
{
  uint32_t serial_size = cam_data_->cam_image_size;
  unsigned char *addr = cam_data_->data + cam_data_->cam_image_offset;
  sensor_msgs::ImagePtr img_(new sensor_msgs::Image());
  ros::serialization::IStream stream((uint8_t *)addr, serial_size);
  ros::serialization::deserialize(stream, *img_);

  return img_;
}

/*
 *
 */
sensor_msgs::PointCloud2Ptr
ImageSharedMem::get_pcl_data()
{
  uint32_t serial_size = cam_data_->pcl_size;
  unsigned char *addr = cam_data_->data + cam_data_->pcl_offset;
  sensor_msgs::PointCloud2Ptr pcl_(new sensor_msgs::PointCloud2());
  ros::serialization::IStream stream((uint8_t *)addr, serial_size);
  ros::serialization::deserialize(stream, *pcl_);
  return pcl_;
}


} /// namespace
