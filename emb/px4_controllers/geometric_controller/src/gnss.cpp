#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/triangle_form.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <controller_msgs/PositionCommand.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <fstream>
#include <yaml-cpp/yaml.h>

#define latA 21.0065177
#define longA 105.8429565

#define latB 21.0065177
#define longB 105.8431188

#define latC 21.0065177
#define longC 105.8432823

#define latD 21.0066968
#define longD 105.8432903

#define latE 21.0066968
#define longE 105.8431183

#define latF 21.0066968
#define longF 105.8429565

#define latG 21.0066968
#define longG 105.8427926

#define latH 21.0065177
#define longH 105.8427972

#define latI 21.0063461
#define longI 105.8427886

#define latJ 21.0063461
#define longJ 105.8429565

double maxj = 5.0 , maxa = 4.0 , maxv = 2.5 , maxav = 1.2 , maxaa = 2.0;
int sample_idx=0,waypoint_idx=0;
int sample_size=0;
TriangleForm plan;
std::vector<int> Indexwp;
bool gps_home_init = false, odom_init = false , setmode_init =false ,plan_init = false ,plan_fin =false;
Eigen::Quaterniond mav_att ;
double mav_yaw = 0,mav_yawvel = 0;
Eigen::Vector3d gps_home,gpsraw,local_start,mav_pos,mav_vel,offset;
std::vector<Eigen::Vector3d> gps_target,local_setpoint;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;

double ToEulerYaw(const Eigen::Quaterniond& q){
    Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

void odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_att.w()=odomMsg.pose.pose.orientation.w;
  mav_att.x()=odomMsg.pose.pose.orientation.x;
  mav_att.y()=odomMsg.pose.pose.orientation.y;
  mav_att.z()=odomMsg.pose.pose.orientation.z;
  mav_yaw = ToEulerYaw(mav_att);
  mav_yawvel = (mav_att * toEigen(odomMsg.twist.twist.angular))(2);
  mav_pos = toEigen(odomMsg.pose.pose.position);
  mav_vel = mav_att * toEigen(odomMsg.twist.twist.linear);
  odom_init = true;
}
double distance( Eigen::Vector3d cur , Eigen::Vector3d fin) {
  Eigen::Vector3d rm = cur-fin;
  return (cur-fin).norm();
}
void gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 
 if(!gps_home_init && odom_init){
 local_start = mav_pos;
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y);//32 zurich 48 VietNam
 for(auto target : gps_target){
    LatLonToUTMXY(target(0),target(1),48,UTM_SP_X,UTM_SP_Y);
    Eigen::Vector3d setpoint;
    setpoint(0) = mav_pos(0) - UTM_X + UTM_SP_X;
    setpoint(1) = mav_pos(1) - UTM_Y + UTM_SP_Y;
    setpoint(2) = target(2);
    setpoint += offset;
    // setpoint(3) = target(3);
    local_setpoint.push_back(setpoint);
 }
 gps_home_init = true; 
 for(auto lsp: local_setpoint){
  ROS_INFO_STREAM("local_setpoint" << lsp(0) << " " << lsp(1));
  }
 }
}

int main(int argc, char **argv) {
  for(int i = 1; i < argc ; i++){ 
    switch(argv[i][0]){
      case 'A': {
        gps_target.push_back(Eigen::Vector3d(latA,longA,5.0));
        break;
      }

      case 'B': {
        gps_target.push_back(Eigen::Vector3d(latB,longB,5.0));
        break;
      }

      case 'C': {
        gps_target.push_back(Eigen::Vector3d(latC,longC,5.0));
        break;
      }

      case 'D': {
        gps_target.push_back(Eigen::Vector3d(latD,longD,7.0));
        break;
      }

      case 'E': {
        gps_target.push_back(Eigen::Vector3d(latE,longE,7.0));
        break;
      }

      case 'F': {
        gps_target.push_back(Eigen::Vector3d(latF,longF,7.0));
        break;
      }

      case 'G': {
        gps_target.push_back(Eigen::Vector3d(latG,longG,7.0));
        break;
      }

      case 'H': {
        gps_target.push_back(Eigen::Vector3d(latH,longH,7.0));
        break;
      }

      case 'I': {
        gps_target.push_back(Eigen::Vector3d(latI,longI,5.0));
        break;
      }

      case 'J': {
        gps_target.push_back(Eigen::Vector3d(latJ,longJ,5.0));
        break;
      }

      default:{
        ROS_ERROR("Invalid command");
        return 0;
      }
    }

  }
 
  int argc_ = 0;
  char ** argv_;

  ros::init(argc_, argv_, "controller_gps");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
   std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
   std::ifstream file(yamlPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << yamlPath << std::endl;
        return 1;
    }
    // Load YAML data from the file
    YAML::Node yamlData = YAML::Load(file);
    // Access the loaded data
    if (yamlData["offsetX"].IsDefined())
    {
        offset(0) = -yamlData["offsetX"].as<double>();
    }
    if (yamlData["offsetX"].IsDefined())
    {
        offset(1) = -yamlData["offsetY"].as<double>();
    }
  file.close();
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/global_position/global", 1, &gpsrawCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher pos_cmd = n.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd",1);
  ros::Publisher acc_cmd = n.advertise<geometry_msgs::Point>("/controller/acc_cmd",1);
  ros::ServiceClient setModeClient = n.serviceClient<geometric_controller::setmode>("/controller/set_mode");
  geometric_controller::setmode setModeCall;

  
  while(ros::ok()){
    loop_rate.sleep();
    if(!setmode_init && gps_home_init){
      setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      setModeCall.request.timeout = 50;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success)
      setmode_init = true;
    }
    if(!plan_init && setmode_init){
       plan.setStart(mav_pos,mav_yaw);
       for(auto lsp : local_setpoint)
       plan.appendSetpoint(lsp);
       plan.execute();
       plan.getWpIndex(Indexwp);
       sample_size = plan.getSize();
       plan_init = true;
    }
    if(plan_init){
      sample_idx ++;
      ROS_INFO_STREAM("Sample idx: " << sample_idx << "Sample size: " << sample_size);
      if(sample_idx < sample_size)
      { 
        if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
        controller_msgs::PositionCommand cmd;
        cmd.position = toGeometry_msgs(plan.getPos(sample_idx));
        cmd.velocity = toVector3(plan.getVel(sample_idx));
        cmd.acceleration = toVector3(plan.getAcc(sample_idx));
        cmd.yaw = plan.getYaw(sample_idx);
        pos_cmd.publish(cmd);
        geometry_msgs::Point acc_msg;
        acc_msg = toGeometry_msgs(plan.getAcc(sample_idx));
        acc_cmd.publish(acc_msg);
        ROS_INFO_STREAM("Sec2 next waypoint " << (Indexwp[waypoint_idx] - sample_idx)*0.01 
        << " Sec to end " << (sample_size - sample_idx) *0.01);
      }
      else{
        setModeCall.request.mode = setModeCall.request.HOLD;
        setModeCall.request.timeout = 50;
        setModeClient.call(setModeCall);
        if(setModeCall.response.success)
        ros::shutdown();
        return 0;
      }
    }
    ros::spinOnce();
  }
   
   return 0;
 }