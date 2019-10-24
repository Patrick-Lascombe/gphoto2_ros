
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

enum CameraState {locked, unlocked};
enum Task {set_focus, trigger_capture, unlock_camera, download_pictures};

class PhotoNodeSim
{
protected:
  ros::NodeHandle nh;

  actionlib::SimpleActionServer<gphoto2_ros::SetFocusAction> as_set_focus;
  actionlib::SimpleActionServer<gphoto2_ros::TriggerAction> as_trigger;
public:

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
  ros::ServiceServer is_camera_ready_srv_;

  boost::mutex photo_mutex_ ;
  ros::Publisher path_pub_;
  ros::Timer picture_path_timer_;

  std::string vendor_id_;
  std::string model_;
  std::string bus_number_;
  std::string port_number_;

  std::string shutter_speed_mode_;
  std::string aperture_mode_;
  std::string iso_mode_;
  CameraState cs_;

  std::vector<Task> tasks_;
  std::vector<std::string> picture_path_list;

  PhotoNodeSim(std::string name_action_set_focus, std::string name_action_trigger) :
      as_set_focus(nh, name_action_set_focus, boost::bind(&PhotoNodeSim::execute_set_focus_CB, this, _1), false),
      as_trigger(nh, name_action_trigger, boost::bind(&PhotoNodeSim::execute_trigger_CB, this, _1), false){
      ros::NodeHandle nh_priv("~");
      ros::NodeHandle nh;

      //Camera filters
      nh_priv.getParam("vendor_id", vendor_id_);
      nh_priv.getParam("model", model_);
      nh_priv.getParam("bus_number", bus_number_);
      nh_priv.getParam("port_number", port_number_);
      //Camera configs
      nh_priv.getParam("shutter_speed_mode", shutter_speed_mode_);
      nh_priv.getParam("aperture_mode", aperture_mode_);
      nh_priv.getParam("iso_mode", iso_mode_);

      // ***** Start Services *****
      set_config_srv_ = nh.advertiseService("set_config", &PhotoNodeSim::setConfig, this);
      get_config_srv_ = nh.advertiseService("get_config", &PhotoNodeSim::getConfig, this);
      capture_srv_ = nh.advertiseService("capture", &PhotoNodeSim::capture, this);
      set_focus_srv_ = nh.advertiseService("set_focus", &PhotoNodeSim::setFocus, this);
      trigger_capture_srv_ = nh.advertiseService("trigger_capture", &PhotoNodeSim::triggerCapture, this);
      unlock_camera_srv_ = nh.advertiseService("unlock_camera", &PhotoNodeSim::unlockCamera, this);
      download_pictures_srv_ = nh.advertiseService("download_pictures", &PhotoNodeSim::downloadPictures, this);
      get_picture_path_list_srv_ = nh.advertiseService("get_picture_path_list", &PhotoNodeSim::getPicturePathList, this);
      reset_picture_path_list_srv_ = nh.advertiseService("reset_picture_path_list", &PhotoNodeSim::resetPicturePathList, this);
      delete_pictures_srv_ = nh.advertiseService("delete_pictures", &PhotoNodeSim::deletePictures, this);
      is_camera_ready_srv_ = nh.advertiseService("is_camera_ready", &PhotoNodeSim::isCameraReady, this);

      path_pub_ = nh.advertise<std_msgs::String>("canon/eos/picture_path", 10);

      as_set_focus.start();
      as_trigger.start();

      // ***** Loop to keep list of taken pictures updated
      picture_path_timer_ = nh.createTimer(ros::Duration(0.01), &PhotoNodeSim::picturePathTimerCallback, this);

      cs_ = unlocked;

  }

  void camera_configs(std::string aperture_mode, std::string shutter_speed_mode, std::string iso_mode){
    ROS_INFO("Settings aperture/shutterspeed/iso: %s/%s/%s", aperture_mode.c_str(), shutter_speed_mode.c_str(), iso_mode.c_str());
  }

  bool setConfig(gphoto2_ros::SetConfig::Request& req, gphoto2_ros::SetConfig::Response& resp) {

  }

  bool getConfig(gphoto2_ros::GetConfig::Request& req, gphoto2_ros::GetConfig::Response& resp) {

  }

  bool capture(gphoto2_ros::Capture::Request& req, gphoto2_ros::Capture::Response& resp) {

  }

  bool setFocus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
      if(cs_ == unlocked) {
          ROS_INFO("Setting the focus");
      } else {
          ROS_INFO("Couldn't set the focus, camera is not unlocked");
      }
  }


  bool triggerCapture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    // Saving the pictures in a specific folder

    cs_ = locked;
    std::vector<char> buffer;
    std::vector<char> *binaryStr;

    ros::Time t = ros::Time::now();
    std::string t_str = boost::lexical_cast<std::string>(t.toSec());
    std::string path = "/tmp/IMG_" + t_str + ".jpeg";
    FILE* outfile = fopen(path.c_str(), "wb");
    fwrite(binaryStr , 1 , sizeof(binaryStr) ,outfile );
    fclose(outfile);

    ROS_INFO("Saving pictures : %s", path.c_str());

    std_msgs::String msg;
    msg.data = path;
    path_pub_.publish(msg);

    resp.success = true;
    return true;

  }

  bool unlockCamera(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    ROS_INFO("Camera Unlocked");
    cs_ = unlocked;
  }

  bool downloadPictures(gphoto2_ros::DownloadPictures::Request& req, gphoto2_ros::DownloadPictures::Response& resp) {
      if (picture_path_list.size() != req.computer_paths.size()){
        ROS_WARN("requested paths list do not match camera path list: picture_path_list size: %d : computer_paths_size: %d", picture_path_list.size(), req.computer_paths.size());
        resp.success=false;
        return  true;
      }

      std::string delimiter = "/", on_camera_folder, on_camera_filename, on_computer_folder, on_computer_filename;

      ros::Time t_begin = ros::Time::now();
      ros::Duration mean_time;
      mean_time.fromSec(0);
      int c=0;
      //Pre treat all the data to get folder and file separated
      for(int i=0; i<picture_path_list.size() ; i++) {
        size_t cam_pos, compu_pos;

        cam_pos = picture_path_list[i].find_last_of('/');
        on_camera_folder = picture_path_list[i].substr(0, cam_pos+1);
        on_camera_filename = picture_path_list[i].substr(cam_pos+1);

        compu_pos = req.computer_paths[i].find_last_of('/');
        on_computer_folder = req.computer_paths[i].substr(0, compu_pos+1);
        on_computer_filename = req.computer_paths[i].substr(compu_pos+1);

        CameraFilePath path;
        std::strcpy(path.name, on_camera_filename.c_str());
        std::strcpy(path.folder, on_camera_folder.c_str());
        ros::Time t_lock = ros::Time::now();

        ROS_INFO_STREAM("Downloading " << path.folder << path.name << " on " << on_computer_folder << on_computer_filename);
        photo_mutex_.lock();
        // Rewrite picture download
        std::string path_camera, path_computer;
        if(!std::experimental::filesystem::exists(req.computer_paths[i])) {
          std::experimental::filesystem::copy(strcat(path.folder, path.name), on_computer_folder + on_computer_filename);
        } else {
          ROS_ERROR("The path to the file you're trying to save already exist");
        }
        //        camera_.download_picture(path, on_computer_folder, on_computer_filename);
        photo_mutex_.unlock();

        ros::Time t_unlock = ros::Time::now();
        mean_time += (t_unlock - t_lock);
        c++;
      }
      ros::Time t_end = ros::Time::now();
      std::cout << "Total duration : " << (t_end - t_begin).toSec() << " for " << c << " pictures" << std::endl;
      std::cout << "Mean lock time per pic : " << mean_time.toSec()/c << std::endl;
      resp.success = true;
      return true;
  }

  bool getPicturePathList(gphoto2_ros::GetPicturePathList::Request& req, gphoto2_ros::GetPicturePathList::Response& resp) {
    resp.picture_path_list=picture_path_list;
    return true;
  }

  bool resetPicturePathList(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
      picture_path_list.clear();
      resp.success = true;
      return true;
  }

  bool deletePictures(gphoto2_ros::DeletePictures::Request& req, gphoto2_ros::DeletePictures::Response& resp) {
      std::vector<std::string>::iterator str_it;

      std::string delimiter = "/", folder, filename;
      for(str_it = req.camera_paths.begin();
          str_it != req.camera_paths.end(); str_it++) {
          std::experimental::filesystem::remove(*str_it);
      }
  }

  void execute_set_focus_CB(const gphoto2_ros::SetFocusGoalConstPtr &goal) {
    if(cs_ == unlocked) {
        ROS_INFO("Setting the focus");
        as_set_focus.setAborted();
    } else {
        ROS_INFO("Couldn't set the focus, camera is not unlocked");
        as_set_focus.setSucceeded();
    }
  }

  void execute_trigger_CB(const gphoto2_ros::TriggerGoalConstPtr &goal) {
    ROS_INFO( "Triggering capture action" );

    cs_ = locked;
    std::vector<char> buffer;
    std::vector<char> *binaryStr;

    ros::Time t = ros::Time::now();
    std::string t_str = boost::lexical_cast<std::string>(t.toNSec());
    std::string path = "/tmp/IMG_" + t_str + ".jpeg";
    FILE* outfile = fopen(path.c_str(), "wb");
//    fwrite(binaryStr , 1 , sizeof(binaryStr) ,outfile );
    fclose(outfile);

    ROS_INFO("Saving pictures : %s", path.c_str());

    std_msgs::String msg;
    msg.data = path;
    picture_path_list.push_back(path);
    path_pub_.publish(msg);
    as_trigger.setSucceeded();
  }

  void picturePathTimerCallback(const ros::TimerEvent&) {

  }

  bool isCameraReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp )
  {
    resp.success = true;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "photo_node");
  ros::AsyncSpinner spinner(2);
  PhotoNodeSim a("set_focus_action", "trigger_action");
  spinner.start();
  ros::waitForShutdown();
  a.~PhotoNodeSim();

}
