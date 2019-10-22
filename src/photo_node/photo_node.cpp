/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <gphoto2_ros/photo_node.h>

PhotoNode::PhotoNode(std::string name_action_set_focus, std::string name_action_trigger) :
  camera_list_(),
  camera_(),
  image_(),
  as_set_focus(nh, name_action_set_focus, boost::bind(&PhotoNode::execute_set_focus_CB, this, _1), false),
  as_trigger(nh, name_action_trigger, boost::bind(&PhotoNode::execute_trigger_CB, this, _1), false)
{

  ros::NodeHandle nh_priv("~");

  //Camera filters
  nh_priv.getParam("owner", owner_);

  //Camera configs
  nh_priv.getParam("shutter_speed_mode", shutter_speed_mode_);
  nh_priv.getParam("aperture_mode", aperture_mode_);
  nh_priv.getParam("iso_mode", iso_mode_);

  ROS_INFO("photo_node: Opening camera with owner field: %s", owner_.c_str());
  ROS_INFO("photo_node: waiting for camera to be plugged or switched on");
  while (!camera_initialization(owner_) && ros::ok()){

    ros::Duration(2.0).sleep();
  }
  ROS_INFO("photo_node: Got camera, starting");
  ROS_INFO("photo_node: configuring");
  camera_configs(aperture_mode_,shutter_speed_mode_, iso_mode_);


  // ***** Start Services *****
  set_config_srv_ = nh.advertiseService("set_config", &PhotoNode::setConfig, this);
  get_config_srv_ = nh.advertiseService("get_config", &PhotoNode::getConfig, this);
  capture_srv_ = nh.advertiseService("capture", &PhotoNode::capture, this);
  set_focus_srv_ = nh.advertiseService("set_focus", &PhotoNode::setFocus, this);
  trigger_capture_srv_ = nh.advertiseService("trigger_capture", &PhotoNode::triggerCapture, this);
  unlock_camera_srv_ = nh.advertiseService("unlock_camera", &PhotoNode::unlockCamera, this);
  download_pictures_srv_ = nh.advertiseService("download_pictures", &PhotoNode::downloadPictures, this);
  get_picture_path_list_srv_ = nh.advertiseService("get_picture_path_list", &PhotoNode::getPicturePathList, this);
  reset_picture_path_list_srv_ = nh.advertiseService("reset_picture_path_list", &PhotoNode::resetPicturePathList, this);
  delete_pictures_srv_ = nh.advertiseService("delete_pictures", &PhotoNode::deletePictures, this);
  is_camera_ready_srv_ = nh.advertiseService("is_camera_ready", &PhotoNode::isCameraReady, this);
  get_config_client_ = nh.serviceClient<gphoto2_ros::GetConfig>("get_config");

  path_pub_ = nh.advertise<std_msgs::String>("canon/eos/picture_path", 10);

  as_set_focus.start();
  as_trigger.start();

  // ***** Loop to keep list of taken pictures updated
  picture_path_timer_ = nh.createTimer(ros::Duration(0.01), &PhotoNode::picturePathTimerCallback, this);
  //picture_path_timer_.stop();
  //reinit_camera_timer_ = nh.createTimer(ros::Duration(10), &PhotoNode::reinitCameraCallback, this);

}


PhotoNode::~PhotoNode()
{
  // shutdown camera
  exit_loop_ = true;
  camera_.photo_camera_close();
}


bool PhotoNode::camera_initialization(std::string desired_owner){
  //ROS_INFO( "photo_node: cam_init" );
  camera_list_=*(new photo_camera_list());
  camera_=*(new photo_camera());
  // create context
  private_context = camera_.photo_camera_create_context();

  // autodetect all cameras connected
  if( camera_list_.autodetect( private_context ) == false )
  {
    ROS_FATAL( "photo_node: Autodetection of cameras failed." );
    gp_context_unref( private_context );
    return false;
  }

  //ROS_INFO_STREAM( "photo_node: before loop  of " << gp_list_count(camera_list_.getCameraList()));
  for (int i=0;i <gp_list_count(camera_list_.getCameraList()); i++) {
    //ROS_INFO_STREAM( "photo_node: loop index: " <<i << "out of " << gp_list_count(camera_list_.getCameraList()));
    if ( !camera_.photo_camera_open(&camera_list_,i)){
      //ROS_WARN_STREAM("photo_node: Could not open camera n " << i);
    }
    else{
      char* value = new char[255];
      bool error_code = camera_.photo_camera_get_config("ownername", &value );
      if( error_code && desired_owner==value)
      {
        current_port_info=camera_.get_port_info();
        ROS_WARN_STREAM("Owner is "<< value << " on port " << current_port_info);
        is_camera_connected_=true;
        camera_list_.~photo_camera_list();
        ROS_INFO("Camera initialized");
        return true;
      }
      delete[] value;
    }
  }

  gp_context_unref( private_context );
  camera_.photo_camera_close();
  camera_list_.~photo_camera_list();
  return false;
}



void PhotoNode::camera_configs(std::string aperture_mode, std::string shutter_speed_mode, std::string iso_mode){
  photo_mutex_.lock();
  ROS_INFO("Settings aperture/shutterspeed/iso: %s/%s/%s", aperture_mode.c_str(), shutter_speed_mode.c_str(), iso_mode.c_str());
  camera_.photo_camera_set_config( "aperture", aperture_mode_ );
  camera_.photo_camera_set_config( "shutterspeed", shutter_speed_mode_ );
  camera_.photo_camera_set_config( "iso", "1" );
  camera_.photo_camera_set_config( "iso", iso_mode_ );

  //Ensure record on SD and clock is sync
  camera_.photo_camera_set_config( "capturetarget", "1" );
  camera_.photo_camera_set_config( "syncdatetimeutc", "0" );
  is_camera_configured_=true;
  photo_mutex_.unlock();
}

void PhotoNode::execute_set_focus_CB(const gphoto2_ros::SetFocusGoalConstPtr &goal)
{
  photo_mutex_.lock();
  bool error_code_focus_drive = camera_.photo_camera_set_config("autofocusdrive", "true");
  ros::Duration(1.5).sleep();
  bool error_code_cancel_focus = camera_.photo_camera_set_config("cancelautofocus", "true");
  photo_mutex_.unlock();
  if (error_code_focus_drive && error_code_cancel_focus){
    as_set_focus.setSucceeded();
  }else {
    as_set_focus.setAborted();
  }
}

void PhotoNode::execute_trigger_CB(const gphoto2_ros::TriggerGoalConstPtr &goal)
{
  std::cout << "Triggering capture action" << std::endl;
  photo_mutex_.lock();
  std::string port_info = camera_.get_port_info();

  bool error_code_trigger = camera_.photo_camera_set_config("eosremoterelease", "5");

  port_info = camera_.get_port_info();

  photo_mutex_.unlock();
  if (error_code_trigger ){
    as_trigger.setSucceeded();
  }else {
    as_trigger.setAborted();
  }
}

std::string PhotoNode::usb_from_vendor_bus_and_port_numbers(std::string bus_number, std::string port_number, std::string id_vendor){
  std::string usb_to_load="";

  // ls usb
  libusb_device **devs;
  int r;
  ssize_t cnt;

  r = libusb_init(NULL);


  cnt = libusb_get_device_list(NULL, &devs);
  if (cnt < 0){
    libusb_exit(NULL);
  }

  //print dev
  libusb_device *dev;
  int i = 0, j = 0;
  uint8_t path[8];

  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
      fprintf(stderr, "failed to get device descriptor");
      return "";
    }

    std::string detected_bus=std::to_string(libusb_get_bus_number(dev));

    r = libusb_get_port_numbers(dev, path, sizeof(path));

    std::string detected_port;
    if (r > 0) {
      detected_port=std::to_string(path[0]);
      for (j = 1; j < r; j++){
        detected_port=detected_port + "." + std::to_string(path[j]);
      }
    }

    std::stringstream stream_id_vendor;
    stream_id_vendor << std::hex << desc.idVendor;
    std::string detected_id_vendor= std::string(4 - stream_id_vendor.str().length(), '0') + stream_id_vendor.str();


    if (detected_bus==bus_number && detected_port==port_number && detected_id_vendor==id_vendor){
      std::string bus_string = std::string(3 - detected_bus.length(), '0') + detected_bus;
      std::string device_string = std::string(3 - std::to_string(libusb_get_device_address(dev)).length(), '0') + std::to_string(libusb_get_device_address(dev));
      usb_to_load=usb_to_load + "usb:" + bus_string + "," + device_string;

      ROS_INFO("usb_to_load: %s, corresponding to bus: %s, port: %s and vendor: %s and device number : %s", usb_to_load.c_str(), detected_bus.c_str(),  detected_port.c_str(), detected_id_vendor.c_str(), device_string.c_str());
    }
  }
  return  usb_to_load;
}

bool PhotoNode::setConfig( gphoto2_ros::SetConfig::Request& req, gphoto2_ros::SetConfig::Response& resp )
{
  photo_mutex_.lock();
  bool error_code = camera_.photo_camera_set_config( req.param, req.value );
  photo_mutex_.unlock();
  return error_code;
}

bool PhotoNode::getConfig( gphoto2_ros::GetConfig::Request& req, gphoto2_ros::GetConfig::Response& resp )
{
  char* value = new char[255];
  photo_mutex_.lock();
  bool error_code = camera_.photo_camera_get_config( req.param, &value );
  if( error_code )
  {
    resp.value = value;
  }
  photo_mutex_.unlock();
  delete[] value;
  return error_code;
}

//Old capture from the original wrapper : Is doing a full press and release, and then download the picture using a non optimal method (Maybe delete this part)
bool PhotoNode::capture( gphoto2_ros::Capture::Request& req, gphoto2_ros::Capture::Response& resp )
{
  // capture a camera image
  photo_mutex_.lock();
  bool error_code = camera_.photo_camera_capture( &image_ );
  if( error_code )
  {
    // fill image message
    fillImage( resp.image, "rgb8", image_.getHeight(), image_.getWidth(), image_.getBytesPerPixel() * image_.getWidth(), image_.getDataAddress() );
  }
  photo_mutex_.unlock();
  return error_code;
}

//Set the focus of the camera, the sleep in the middle of the function is necessary to give the camera time to focus properly
bool PhotoNode::setFocus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp )
{
  photo_mutex_.lock();
  bool error_code_focus_drive = camera_.photo_camera_set_config("autofocusdrive", "true");
  ros::Duration(1.5).sleep();
  bool error_code_cancel_focus = camera_.photo_camera_set_config("cancelautofocus", "true");
  photo_mutex_.unlock();
  resp.success = true;
  return true;
}

//Take instantaneously a picture and save it in the memory card (be sure that capturetarget = 1 in the camera config)
bool PhotoNode::triggerCapture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
  std::cout << "Triggering capture service" << std::endl;
  photo_mutex_.lock();
  bool error_code_focus_drive = camera_.photo_camera_set_config("eosremoterelease", "5");
  photo_mutex_.unlock();
  resp.success = true;
  return true;
}

//After taking pictures using triggerCapture the camera is locked in a state, this function unlock the camera and allow us to execute other actions
bool PhotoNode::unlockCamera(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
  photo_mutex_.lock();
  bool error_code_focus_drive = camera_.photo_camera_set_config("eosremoterelease", "11");
  photo_mutex_.unlock();
  resp.success = true;
  return true;
}

//Downaload all the pictures in the contained in the req.camera_paths field (complete paths are necessary)
// into a folder precised in req.computer_path
bool PhotoNode::downloadPictures(gphoto2_ros::DownloadPictures::Request& req, gphoto2_ros::DownloadPictures::Response& resp) {

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
    if(!std::experimental::filesystem::exists(req.computer_paths[i])) {
      photo_mutex_.lock();
      camera_.download_picture(path, on_computer_folder, on_computer_filename);
      photo_mutex_.unlock();
    } else {
      ROS_ERROR("The file already exist, picture won't be saved");
    }

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

bool PhotoNode::getPicturePathList(gphoto2_ros::GetPicturePathList::Request& req, gphoto2_ros::GetPicturePathList::Response& resp) {
  resp.picture_path_list=picture_path_list;
  return true;
}

bool PhotoNode::resetPicturePathList(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp )
{
  picture_path_list.clear();
  resp.success = true;
  return true;
}

bool PhotoNode::isCameraReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp )
{
  resp.success = is_camera_connected_ && is_camera_configured_;
  return true;
}

bool PhotoNode::deletePictures(gphoto2_ros::DeletePictures::Request &req, gphoto2_ros::DeletePictures::Response &resp) {
  std::vector<std::string>::iterator str_it;

  std::string delimiter = "/", folder, filename;

  ros::Time t_begin = ros::Time::now();
  ros::Duration mean_time;
  mean_time.fromSec(0);
  //Pre treat all the data to get folder and file separated
  for(str_it = req.camera_paths.begin();
      str_it != req.camera_paths.end(); str_it++) {
    size_t pos;

    pos = str_it->find_last_of('/');
    folder = str_it->substr(0, pos+1);
    filename = str_it->substr(pos+1);

    CameraFilePath path;
    std::strcpy(path.name, filename.c_str());
    std::strcpy(path.folder, folder.c_str());
    camera_.delete_pictures(path);
  }
  resp.success = true;
  return true;
}

// This service must be running in a different thread all the time to recover the paths of the pictures that are taken
// it listen to the events coming from the camera on a loop, and save the path of the picture taken when the right events is coming
void PhotoNode::picturePathTimerCallback(const ros::TimerEvent&) {
  picturePathCheck();
}

void PhotoNode::picturePathCheck() {
  std::string path_to_file = camera_.get_picture_path(&photo_mutex_);
  if(path_to_file != "") {
    ROS_INFO("Adding picture path to list: %s", path_to_file.c_str());
    picture_path_list.push_back(path_to_file);
    std_msgs::String msg;
    msg.data = path_to_file;
    path_pub_.publish(msg);
  }
}

void PhotoNode::reinitCameraCallback(const ros::TimerEvent &) {

}

bool PhotoNode::isDeviceExist( std::string port_info){
  bool is_device_found =false;

  int bus_to_find=std::stoi(port_info.substr (4,3));
  int device_to_find=std::stoi(port_info.substr (8,3));

  libusb_device **list;
  libusb_device *found = NULL;
  ssize_t cnt = libusb_get_device_list(NULL, &list);
  ssize_t i = 0;
  int err = 0;
  if (cnt < 0)
    libusb_exit(NULL);
  for (i = 0; i < cnt; i++) {
    libusb_device *device = list[i];
    int bus_nb=libusb_get_bus_number(device);
    int device_nb= libusb_get_device_address(device);
    //ROS_INFO_STREAM("Testing: Bus: " << bus_nb << ", Device: " << device_nb);
    if ((bus_nb == bus_to_find) && (device_nb == device_to_find)){
      //ROS_INFO_STREAM("Found: Bus: " << bus_nb << ", Device: " << device_nb);
      is_device_found=true;
      break;
    }
  }
  libusb_free_device_list(list, 1);
  return is_device_found;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "photo_node");
  ros::AsyncSpinner spinner(4);
  PhotoNode a("set_focus_action", "trigger_action");
  spinner.start();


  ros::Rate r(10);
  r.sleep();
  while (ros::ok()) {
    if (a.is_camera_connected_){
      //ROS_INFO("Main: Testing cam connection");
      if (a.isDeviceExist(a.current_port_info)){
        //ROS_WARN_STREAM("Main: cam ok");
        //a.picturePathCheck();
      }else {
        ROS_WARN_STREAM("Main: cam disconnected");
        a.picture_path_timer_.stop();
        a.camera_.photo_camera_close();
        a.is_camera_connected_=false;
        a.is_camera_configured_=false;
        a.current_port_info="";
      }
    }else {
      //ROS_WARN_STREAM("Main :Attempting to reconnect");
      if (a.camera_initialization(a.owner_)){
        ROS_WARN_STREAM("Main: camera reconnected, reconfiguring");
        a.camera_configs(a.aperture_mode_,a.shutter_speed_mode_, a.iso_mode_);
        a.picture_path_timer_.start();
      }else {
        ros::Duration(2.0).sleep();
      }
    }
    r.sleep();
  }
  ros::waitForShutdown();
  a.~PhotoNode();

}
