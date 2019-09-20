#ifndef PHOTO_NODE_H
#define PHOTO_NODE_H

// ROS Headers
#include <ros/ros.h>
#include <self_test/self_test.h>

// ROS Messages
#include <sensor_msgs/fill_image.h>
#include <std_msgs/String.h>

// ROS Services
#include <gphoto2_ros/GetConfig.h>
#include <gphoto2_ros/SetConfig.h>
#include <gphoto2_ros/Capture.h>
#include <gphoto2_ros/DownloadPictures.h>
#include <gphoto2_ros/GetPicturePathList.h>
#include <gphoto2_ros/DeletePictures.h>
#include <std_srvs/Trigger.h>

// photo library headers
#include "gphoto2_ros/photo_camera_list.hpp"
#include "gphoto2_ros/photo_camera.hpp"
#include "gphoto2_ros/photo_image.hpp"
#include "gphoto2_ros/photo_reporter.hpp"

enum Task {set_focus, trigger_capture, unlock_camera, download_pictures};

class PhotoNode
{
public:
  photo_camera_list camera_list_;
  photo_camera camera_;
  photo_image image_;

  boost::mutex photo_mutex_ ;

  ros::ServiceServer set_config_srv_;
  ros::ServiceServer get_config_srv_;
  ros::ServiceServer capture_srv_;
  ros::ServiceServer set_focus_srv_;
  ros::ServiceServer trigger_capture_srv_;
  ros::ServiceServer unlock_camera_srv_;
  ros::ServiceServer download_pictures_srv_;
  ros::ServiceServer get_picture_path_list_srv_;
  ros::ServiceServer reset_picture_path_list_srv_;
  ros::ServiceServer delete_pictures_srv_;

  ros::Publisher path_pub_;
  ros::Timer picutre_path_timer_;
  ros::Timer reinit_camera_timer_;

  std::string usb_;
  std::string model_;

  std::vector<Task> tasks_;
  std::vector<std::string> picture_path_list;

  bool exit_loop_;

  bool is_initialized;

  PhotoNode();
  ~PhotoNode();
  bool setConfig(gphoto2_ros::SetConfig::Request& req, gphoto2_ros::SetConfig::Response& resp);
  bool getConfig( gphoto2_ros::GetConfig::Request& req, gphoto2_ros::GetConfig::Response& resp);
  bool capture( gphoto2_ros::Capture::Request& req, gphoto2_ros::Capture::Response& resp );
  bool setFocus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool triggerCapture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool unlockCamera(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool downloadPictures(gphoto2_ros::DownloadPictures::Request& req, gphoto2_ros::DownloadPictures::Response& resp);
  bool getPicturePathList(gphoto2_ros::GetPicturePathList::Request& req, gphoto2_ros::GetPicturePathList::Response& resp);
  bool resetPicturePathList(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool deletePictures(gphoto2_ros::DeletePictures::Request& req, gphoto2_ros::DeletePictures::Response& resp);
  void picturePathTimerCallback(const ros::TimerEvent&);
  void reinitCameraCallback(const ros::TimerEvent&);
  std::string getDeviceNumber(int cam_number);
};
#endif // PHOTO_NODE_H
