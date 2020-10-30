/**
 * nodelet to save image message
 * Copyright(C) 2020 Isao Hara
 * License: Apache License v2.0
 */
#include <shm_image_save/plugin_nodelet_shm_saver.h>

namespace shm_image_save
{
/*
  Map a shared memory for camera images
*/
cam_shm_data *
map_cam_shm(int *shmid, int id, int size)
{
  void *res;
  res=map_shared_mem(shmid, id, size, 0);
  if(res == NULL){
    res=map_shared_mem(shmid, id, size, 1);
    if (res < 0){
      fprintf(stderr, "Fail to map cam_shm_data\n");
      return NULL;
    }
    memset(res, 0, size);
  }
  return (cam_shm_data *)res;
}

/*
 * Constructor
 */
void
plugin_nodelet_shm_saver::onInit()
{
  nh_ = getNodeHandle();
  setup_shm();

  if (m_cam_ != NULL){
    cam_info_sub_ = nh_.subscribe(CAMERA_INFO, 1,
                       &plugin_nodelet_shm_saver::callbackCameraInfo,this);
    cam_image_sub_ = nh_.subscribe(CAMERA_IMAGE, 1,
                       &plugin_nodelet_shm_saver::callbackColorImage,this);
    rs_cloud_sub_ = nh_.subscribe(PCL_MESSAGE, 1,
                       &plugin_nodelet_shm_saver::callbackRsCloud,this);
    ROS_INFO("Initialize shm_saver nodelet");
  }
  return;
}

void
plugin_nodelet_shm_saver::setup_shm()
{
  ros::param::param<int>("/shm_image_save/shm_id", m_id_, SHM_IMAGE_ID);
  ros::param::param<int>("/shm_image_save/shm_size", m_size_, SHM_IMAGE_LEN);
  ros::param::param<int>("/shm_image_save/max_count", m_max_count_, MAX_MSGS);
  m_cam_ = map_cam_shm(&m_shmid_, m_id_, m_size_);
  if (m_cam_ != NULL){
    m_cam_->max_count = m_max_count_;
  }
  return;
}

/**
 * callback to copy camera_info
 */
void 
plugin_nodelet_shm_saver::callbackCameraInfo(
      const sensor_msgs::CameraInfoConstPtr &msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  m_cam_->cam_info_size = serial_size;
  int pos = (m_cam_->cam_info_count+1) % m_max_count_;
  int offset = m_cam_->cam_info_size * pos;

  memcpy(m_cam_->data + offset, (void *)buffer.get(), serial_size);

  m_cam_->cam_info_offset = offset;
  m_cam_->cam_info_count = pos;
  return;
}

/**
 *
 * callback to copy image data
 */
void 
plugin_nodelet_shm_saver::callbackColorImage(
   const sensor_msgs::ImageConstPtr& msg)

{
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  m_cam_->cam_image_size = serial_size;
  int pos = (m_cam_->cam_image_count+1) % m_max_count_;
  int offset = m_cam_->cam_info_size * m_max_count_ 
               + m_cam_->cam_image_size * pos;

  memcpy(m_cam_->data + offset, (void *)buffer.get(), serial_size);

  m_cam_->cam_image_offset = offset;
  m_cam_->cam_image_count = pos;
  return;
}

/**
 *
 * callback to copy PointCloud2 data
 */
void 
plugin_nodelet_shm_saver::callbackRsCloud(
   const sensor_msgs::PointCloud2ConstPtr &msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  m_cam_->pcl_size = serial_size;

  int pos = (m_cam_->pcl_count+1) % m_max_count_;
  int offset = m_cam_->cam_info_size * m_max_count_ 
               + m_cam_->cam_image_size * m_max_count_
               + m_cam_->pcl_size * pos;

  memcpy(m_cam_->data + offset, (void *)buffer.get(), serial_size);

  m_cam_->pcl_offset = offset ;
  m_cam_->pcl_count = pos;
  return;
}

}

PLUGINLIB_EXPORT_CLASS(shm_image_save::plugin_nodelet_shm_saver, nodelet::Nodelet);

