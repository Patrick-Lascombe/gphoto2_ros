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

//ROS action
#include <actionlib/server/simple_action_server.h>
#include <gphoto2_ros/SetFocusAction.h>
#include <gphoto2_ros/TriggerAction.h>

// photo library headers
#include "gphoto2_ros/photo_camera_list.hpp"
#include "gphoto2_ros/photo_camera.hpp"
#include "gphoto2_ros/photo_image.hpp"
#include "gphoto2_ros/photo_reporter.hpp"

//for usb listing
#include <stdio.h>
#include <libusb-1.0/libusb.h>

#include <experimental/filesystem>

enum Task {set_focus, trigger_capture, unlock_camera, download_pictures};

class PhotoNode
{
protected:
  ros::NodeHandle nh;

  actionlib::SimpleActionServer<gphoto2_ros::SetFocusAction> as_set_focus;
  actionlib::SimpleActionServer<gphoto2_ros::TriggerAction> as_trigger;
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
  ros::ServiceServer is_camera_ready_;

  ros::ServiceClient get_config_client_;

  ros::Publisher path_pub_;
  ros::Timer picutre_path_timer_;
  ros::Timer reinit_camera_timer_;

  std::string vendor_id_;
  std::string model_;
  std::string bus_number_;
  std::string port_number_;

  std::string shutter_speed_mode_;
  std::string aperture_mode_;
  std::string iso_mode_;

  std::vector<Task> tasks_;
  std::vector<std::string> picture_path_list;

  bool exit_loop_;
  bool is_camera_connected_, is_camera_configured_;

  GPContext* private_context;

  PhotoNode(std::string name_action_set_focus, std::string name_action_trigger);
  ~PhotoNode();
  void camera_configs(std::string aperture_mode, std::string shutter_speed_mode, std::string iso_mode);
  void execute_set_focus_CB(const gphoto2_ros::SetFocusGoalConstPtr &goal);
  void execute_trigger_CB(const gphoto2_ros::TriggerGoalConstPtr &goal);
  bool camera_initialization();
  std::string usb_from_vendor_bus_and_port_numbers(std::string bus_number, std::string port_number, std::string id_vendor);
  bool setConfig(gphoto2_ros::SetConfig::Request& req, gphoto2_ros::SetConfig::Response& resp);
  bool getConfig( gphoto2_ros::GetConfig::Request& req, gphoto2_ros::GetConfig::Response& resp);
  bool capture( gphoto2_ros::Capture::Request& req, gphoto2_ros::Capture::Response& resp );
  bool setFocus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool triggerCapture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool unlockCamera(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool downloadPictures(gphoto2_ros::DownloadPictures::Request& req, gphoto2_ros::DownloadPictures::Response& resp);
  bool getPicturePathList(gphoto2_ros::GetPicturePathList::Request& req, gphoto2_ros::GetPicturePathList::Response& resp);
  bool resetPicturePathList(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool isCameraReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool deletePictures(gphoto2_ros::DeletePictures::Request& req, gphoto2_ros::DeletePictures::Response& resp);
  void picturePathTimerCallback(const ros::TimerEvent&);
  void reinitCameraCallback(const ros::TimerEvent&);
};
#endif // PHOTO_NODE_H
