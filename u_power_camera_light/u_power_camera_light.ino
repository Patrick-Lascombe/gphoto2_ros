#include <ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

const int PIN_POWER_CAMERA=0;
const int PIN_POWER_LIGHT=1;

bool isPowerCameraOn=false;
bool isPowerLightOn=false;

ros::NodeHandle  nh;



void set_power_camera(bool on){
  if (on){
    digitalWrite(PIN_POWER_CAMERA, LOW);
    isPowerCameraOn=true;
  }else{
    digitalWrite(PIN_POWER_CAMERA, HIGH);
    isPowerCameraOn=false;
  }
}

void set_power_light(bool on){
  if (on){
    digitalWrite(PIN_POWER_LIGHT, LOW);
    isPowerLightOn=true;
  }else{
    digitalWrite(PIN_POWER_LIGHT, HIGH);
    isPowerLightOn=false;
  }
}


void doSetPowerCameraCb(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res){
  set_power_camera(req.data);
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
  do_set_power_camera_server("do_set_power_camera",&doSetPowerCameraCb);

void doSetPowerLightCb(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res){
  set_power_light(req.data);
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
  do_set_power_light_server("do_set_power_light",&doSetPowerLightCb);
  
  
void setup()
{


  pinMode(PIN_POWER_CAMERA, OUTPUT);
  digitalWrite(PIN_POWER_CAMERA, HIGH);
  pinMode(PIN_POWER_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_LIGHT, HIGH);

  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.advertiseService(do_set_power_camera_server);
  nh.advertiseService(do_set_power_light_server);
}

void loop()
{
  nh.spinOnce();
  if (!nh.connected()){
    if (isPowerCameraOn){
      set_power_camera(false);
    }
    if (isPowerLightOn){
      set_power_light(false);
    }
  }
  delay(10);
}
