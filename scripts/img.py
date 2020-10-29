#
# Test class
# Shared Memory Library
#  Copyright(C) 2020 Isao Hara
#  License: Apache License v2.0
#  
import cv2
import traceback
import shm
###################################
import sensor_msgs
import cv_bridge

#
#
class ImageShm(shm.SharedMem):
  def __init__(self, id=234567, create=None):
    super(ImageShm, self).__init__(id, create)
    max_count = 2

    self.shm_offset={
       'cam_info_size': 0, 'cam_image_size': 4, 'pcl_size': 8,
       'cam_info_offset': 12, 'cam_image_offset': 16, 'pcl_offset': 20,
       'cam_info_count': 24, 'cam_image_count': 28, 'pcl_count': 32,
       'data': 36 
      } 

    self.bridge = cv_bridge.CvBridge()
    self.image = sensor_msgs.msg.Image()
    self.pcl = sensor_msgs.msg.PointCloud2()
  #
  #
  def get_image_size(self):
      ret = self.get_int('cam_image_size')
      return ret
  #
  #
  def get_image_offset(self):
      return self.get_int('cam_image_offset')
  #
  #
  def get_image_data(self):
      pos = self.shm_offset['data'] + self.get_int('cam_image_offset')
      return self.get_shm_bytes(pos, self.get_image_size())
  #
  #
  def get_image(self):
      msg_data = self.get_image_data()
      msg = self.image.deserialize(msg_data)
      try:
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return cv_img
      except:
        traceback.print_exc()
        return None
  #
  #
  def get_pcl_size(self):
      ret = self.get_int('pcl_size')
      return ret
  #
  #
  def get_pcl_offset(self):
      return self.get_int('pcl_offset')
  #
  #
  def get_pcl_data(self):
      pos = self.shm_offset['data'] + self.get_int('pcl_offset')
      return self.get_shm_bytes(pos, self.get_pcl_size())
  #
  #
  def get_pcl_msg(self):
      msg_data = self.get_pcl_data()
      msg = self.pcl.deserialize(msg_data)
      return msg

def main1():
  imgshm = ImageShm()
  
  while 1:
    img = imgshm.get_image()
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)
  cv2.destroyAllWindows()

def main2():
  imgshm = ImageShm()
  msg = imgshm.get_pcl_msg()
  print(msg.header)

#
#
if __name__ == '__main__':
  main1()
 
