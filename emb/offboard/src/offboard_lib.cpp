#include "offboard/offboard.h"
// #include "geometric_controller/geometric_controller.h"
// OffboardControl::OffboardControl(){
// }

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private),
                                                                                                                    //   simulation_mode_enable_(false),
                                                                                                                    //   delivery_mode_enable_(false),
                                                                                                                      return_home_mode_enable_(false) {
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    gps_position_sub_ = nh_.subscribe("/mavros/global_position/global", 1, &OffboardControl::gpsrawCallback, this);
    opt_point_sub_ = nh_.subscribe("optimization_point", 10, &OffboardControl::optPointCallback, this);
    // point_target_sub_ = nh_.subscribe("point_target",10, &OffboardControl::targetPointCallback, this);
    check_last_opt_sub_ = nh_.subscribe("check_last_opt_point",10, &OffboardControl::checkLastOptPointCallback, this);

    //DuyNguyen
    marker_p_sub_ = nh_.subscribe("/target_pos",10, &OffboardControl::markerCallback, this);
    check_move_sub_ = nh_.subscribe("/move_position",10, &OffboardControl::checkMoveCallback, this);
    ids_detection_sub_ = nh_.subscribe("/ids_detection",10, &OffboardControl::checkIdsDetectionCallback, this);
    local_p_sub_ = nh_.subscribe("/mavros/local_position/pose",10, &OffboardControl::poseCallback, this);
    pos_cmd_ = nh_.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd", 1);
    local_sp_vector_pub_ = nh_.advertise<geometry_msgs::PoseArray>("local_sp_vector3d_array", 1);
    // local_sp_num_pub_ = nh_.advertise<std_msgs::Int32>("local_sp_num",1); 
    // odom_error_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_error", 1, true);
    setModeClient = nh_.serviceClient<geometric_controller::setmode>("/controller/set_mode");
    nh_private_.getParam("/offboard_node/target_error", target_error_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
    std::ifstream file(yamlPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << yamlPath << std::endl;
        // return 1;
    }
    // Load YAML data from the file
    YAML::Node yamlData = YAML::Load(file);
    // Access the loaded data
    if (yamlData["offsetX"].IsDefined())
    {
        offset_(0) = -yamlData["offsetX"].as<double>();
    }
    if (yamlData["offsetX"].IsDefined())
    {
        offset_(1) = -yamlData["offsetY"].as<double>();
    }
     file.close();

    waitForPredicate(10.0);
    if (input_setpoint) {
        // inputSetpoint();
        inputPlannerAndLanding();
    }
}

OffboardControl::~OffboardControl() {

}

/* wait for connect, GPS received, ...
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForPredicate(double hz) {
    ros::Rate rate(hz);

    std::printf("\n[ INFO] Waiting for FCU connection \n");
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");

    std::printf("[ INFO] Waiting for GPS signal \n");
    while (ros::ok() && !gps_received_) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] GPS position received \n");
    operation_time_1_ = ros::Time::now();
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_att_.w()=odomMsg.pose.pose.orientation.w;
  mav_att_.x()=odomMsg.pose.pose.orientation.x;
  mav_att_.y()=odomMsg.pose.pose.orientation.y;
  mav_att_.z()=odomMsg.pose.pose.orientation.z;
  mav_yaw_ = ToEulerYaw(mav_att_); 
  mav_yawvel_ = (mav_att_ * toEigen(odomMsg.twist.twist.angular))(2);
  mav_pos_ = toEigen(odomMsg.pose.pose.position);
  current_odom_ = odomMsg;
  mav_vel_ = mav_att_ * toEigen(odomMsg.twist.twist.linear);
//   cmd_.yaw_dot = mav_yawvel_;
//   odom_init = true;
  odom_received_ = true;
}

void OffboardControl::gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 
 if(!gps_received_ && odom_received_){
 local_start_ = mav_pos_;
 local_start_ += offset_;
 gpsraw_(0) = msg.latitude;
 gpsraw_(1) = msg.longitude;
 gpsraw_(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw_(0),gpsraw_(1),48,UTM_X_,UTM_Y_);//32 zurich 48 VietNam
 local_setpoint_.push_back(local_start_); //push back the current position to loal_setpoint_
 for(auto target : gps_target_){
    // int x = gps_target_.capacity(); 
    LatLonToUTMXY(target(0),target(1),48,UTM_SP_X_,UTM_SP_Y_);
    Eigen::Vector3d setpoint;
    setpoint(0) = mav_pos_(0) - UTM_X_ + UTM_SP_X_;
    setpoint(1) = mav_pos_(1) - UTM_Y_ + UTM_SP_Y_;
    setpoint(2) = mav_pos_(2);
    setpoint += offset_;
    // setpoint(3) = target(3);
    local_setpoint_.push_back(setpoint);
 }
 gps_home_init = true; 
 for(auto lsp: local_setpoint_){
  ROS_INFO_STREAM("local_setpoint" << lsp(0) << " " << lsp(1));
  }
 }
}

void OffboardControl::optPointCallback(const geometry_msgs::Point::ConstPtr &msg) {
    // opt_point_ = *msg;
    // opt_point_ = *msg;
    opt_point_ = *msg;
    //optimization_point_.push_back(*msg);
    opt_point_received_ = true;
    // ROS_INFO_STREAM("Hello World");
}

// void OffboardControl::targetPointCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
//     target_array_ = *msg;
// }

void OffboardControl::checkLastOptPointCallback(const std_msgs::Bool::ConstPtr &msg) {
    check_last_opt_point_.data = false;
    check_last_opt_point_ = *msg;
}

void OffboardControl::markerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    marker_position_ = *msg;
}

void OffboardControl::checkMoveCallback(const std_msgs::Bool msg) {
    check_mov_ = msg.data;
}

void OffboardControl::checkIdsDetectionCallback(const std_msgs::Bool msg) {
    check_ids_ = msg.data;
}

void OffboardControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
    current_position_ = *msg;
    // ROS_INFO_STREAM("Hello World");
}

void OffboardControl::inputPlannerAndLanding() {
    ros::Rate rate(10.0);
    std::printf("\n[ INFO] Mission with planner and marker Mode\n");
    // std::printf("[ INFO] Parameter 'return_home_mode_enable' is set %s\n", return_home_mode_enable_ ? "true" : "false");
    // std::printf("[ INFO] Parameter 'delivery_mode_enable' is set %s\n", delivery_mode_enable_ ? "true" : "false");
    std::cout << "Input the number of gps points: " << std::endl;
    // int num;
    std::cin >> num_of_enu_target_;
    int i = 0;
    for(auto target: gps_target_) {
        std::cin >> target(0) >> target(1);
        gps_target_.push_back(target);
        i++;
        ros::spinOnce();
        rate.sleep();
        if(i==num_of_enu_target_) {
            break;
        }
    }
    num_of_enu_target_ += 1;
    num_of_enu_msg_.data = num_of_enu_target_;
    geometry_msgs::PoseArray posearr_target;
    posearr_target = local3dspConvert(local_setpoint_);
	local_sp_vector_pub_.publish(posearr_target);

    std::printf("[ INFO] Loaded global path setpoints: \n");
    for(auto local_sp : local_setpoint_) {
        std::cout << "Setpoint " << local_sp(0) << ", " << local_sp(1) << ", " << local_sp(2) << std::endl;
    }
    std::printf("\n[ INFO] Flight with Planner setpoint\n");
    // std::printf("\n[ INFO] Flight to the start point of Optimization path\n");
    plannerAndLandingFlight();
}

void OffboardControl::plannerAndLandingFlight() {
    bool first_target_reached = false;
    bool stop = false;
    bool final_reached = false;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(50.0);
    //double curr_alpha_hover;
    bool first_receive_hover = true;
    double target_alpha,this_loop_alpha;
    yaw_rate_ = 0.15;
    first_target_reached = false;
    bool target_reached = false; 
    ros::Time current_time = ros::Time::now();
    if (opt_point_received_) {
        std::printf("\n[ INFO] Fly with optimization points\n");  
        bool first_receive = true;
        double target_alpha, this_loop_alpha;
        bool flag = true;
        int k = local_setpoint_.size();
        int l = 0;
        ros::Time t_check;
        t_check = ros::Time::now();
        while (ros::ok()) {
            Eigen::Vector3d x = local_setpoint_[l+1];
            setpoint = targetTransfer(local_setpoint_[k-1](0), local_setpoint_[k-1](1), local_setpoint_[k-1](2));
            // target_reached = checkPlanError(target_error_, x);
            if(target_reached) {
                if((ros::Time::now() - t_check) < ros::Duration(10)) {
                    while(ros::ok()) {
                        setModeCall.request.mode = setModeCall.request.HOLD;
                        setModeCall.request.sub = z_takeoff_; 
                        setModeCall.request.timeout = 50;
                        setModeClient.call(setModeCall);
                        ros::spinOnce();
                        rate.sleep();
                    }
                }
                else {
                    while(ros::ok()) {
                        setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
                        setModeClient.call(setModeCall);
                        if(setModeCall.response.success) {
                            break;
                        }
                    }

                }
            }
            target_reached = checkPlanError(target_error_, x);
            while(ros::ok() && !target_reached) {
                rate.sleep();
                target_alpha = calculateYawOffset(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(opt_point_.x, opt_point_.y, opt_point_.z));
                if ((yaw_ - target_alpha) >= PI) {
                    target_alpha += 2*PI;
                }
                else if ((yaw_ - target_alpha) <= -PI) {
                    target_alpha -= 2*PI;
                }
                else{}

                // calculate the input for position controller (this_loop_alpha) so that the input yaw value will always be higher or lower than current yaw angle (yaw_) a value of yaw_rate_
                // this make the drone yaw slower
                if (target_alpha <= yaw_) {
                    if ((yaw_ - target_alpha) > yaw_rate_) {
                        this_loop_alpha = yaw_ - yaw_rate_;
                    }
                    else {
                        this_loop_alpha = target_alpha;
                    }
                }
                else {
                    if ((target_alpha - yaw_) > yaw_rate_) {
                        this_loop_alpha = yaw_ + yaw_rate_;
                    }
                    else {
                        this_loop_alpha = target_alpha;
                    }
                }

                cmd_.position.x = opt_point_.x; 
                cmd_.position.y = opt_point_.y; 
                cmd_.position.z = opt_point_.z;
                cmd_.yaw = this_loop_alpha;

                if((abs(opt_point_.x - current_odom_.pose.pose.position.x) < 0.3) && (abs(opt_point_.y - current_odom_.pose.pose.position.y) < 0.3)) {
                    // target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
                    cmd_.yaw = yaw_;
                }
                else {
                    // target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(this_loop_alpha);
                    cmd_.yaw = this_loop_alpha;
                }
                
                cmd_.header.stamp = ros::Time::now();
                pos_cmd_.publish(cmd_);
                target_reached = checkPlanError(target_error_, x);
                if(target_reached) {
                    l++;
                    // break;
                }
                ros::spinOnce();
            }    

            final_reached = checkPositionError(target_error_, setpoint);

            if (final_reached && check_last_opt_point_.data) {
                while(ros::ok()) {
                        setModeCall.request.mode = setModeCall.request.HOLD;
                        setModeCall.request.sub = z_takeoff_; 
                        setModeCall.request.timeout = 50;
                        setModeClient.call(setModeCall);
                        ros::spinOnce();
                        rate.sleep();
                    }
            }
            ros::spinOnce();
            rate.sleep();
            // if (flag == false) {
            //     stop = true;
            // }
            //stop = true;
        }
    }
    else {
        std::printf("\n[ WARN] Not received optimization points!\n");
        while(ros::ok()) {
                        setModeCall.request.mode = setModeCall.request.HOLD;
                        setModeCall.request.sub = z_takeoff_; 
                        setModeCall.request.timeout = 50;
                        setModeClient.call(setModeCall);
                        ros::spinOnce();
                        rate.sleep();
                    }
    }
}

//transfer a x,y,z to a poseStamped datatype
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z, geometry_msgs::Quaternion yaw) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = yaw;
    return target;
}

/* transfer x, y, z (meter) and yaw (degree) setpoint to same message type with enu setpoint msg
   input: x, y, z in meter and yaw in degree that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = tf::createQuaternionMsgFromYaw(radianOf(yaw));
    return target;
}

/* transfer x, y, z setpoint to same message type with enu setpoint msg
   input: x, y, z that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z) {
    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    //target.pose.orientation = 0;
    return target;
}

/* perform hover task
   input: setpoint to hover and hover time */
void OffboardControl::hovering(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    ros::Time t_check;

    std::printf("\n[ INFO] Hovering at [%.1f, %.1f, %.1f] in %.1f (s)\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z, hover_time);
    t_check = ros::Time::now();
    while ((ros::Time::now() - t_check) < ros::Duration(hover_time)) {
        cmd_.position.x = setpoint.pose.position.x;
        cmd_.position.y = setpoint.pose.position.y;
        cmd_.position.z = setpoint.pose.position.z;
        // cmd_.yaw = tf::getYaw(setpoint.pose.orientation);
        pos_cmd_.publish(cmd_);
        // setpoint_pose_pub_.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }
}

double OffboardControl::distancePosCmdBetween(controller_msgs::PositionCommand target) {
    Eigen::Vector3d distance;
    distance << target.position.x - current_odom_.pose.pose.position.x,
        target.position.y - current_odom_.pose.pose.position.y,
        target.position.z - current_odom_.pose.pose.position.z;

    return distance.norm();
}

/* check offset between current position from odometry and setpoint position to decide when drone reached setpoint
   input: error in meter to check and target pose. This function check between current pose from odometry and target pose */
bool OffboardControl::checkPositionError(double error, geometry_msgs::PoseStamped target) {
    Eigen::Vector3d geo_error;
    geo_error << target.pose.position.x - current_odom_.pose.pose.position.x, target.pose.position.y - current_odom_.pose.pose.position.y, target.pose.position.z - current_odom_.pose.pose.position.z;

    return (geo_error.norm() < error) ? true : false;
}

bool OffboardControl::checkPosCmdError(double error, controller_msgs::PositionCommand target) {
    Eigen::Vector3d geo_error;
    geo_error << target.position.x - current_odom_.pose.pose.position.x, target.position.y - current_odom_.pose.pose.position.y, target.position.z - current_odom_.pose.pose.position.z;
    return (geo_error.norm() < error) ? true : false;
}

/* calculate yaw offset between current position and next optimization position */
double OffboardControl::calculateYawOffset(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped setpoint) {
    double alpha;
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double xs = setpoint.pose.position.x;
    double ys = setpoint.pose.position.y;

    alpha = atan2(abs(ys - yc), abs(xs - xc));
    if ((xs > xc) && (ys > yc)) {
        return alpha;
    }
    else if ((xs < xc) && (ys > yc)) {
        return (PI - alpha);
    }
    else if ((xs < xc) && (ys < yc)) {
        return (alpha - PI);
    }
    else if ((xs > xc) && (ys < yc)) {
        return (-alpha);
    }
    else if ((xs == xc) && (ys > yc)) {
        return alpha;
    }
    else if ((xs == xc) && (ys < yc)) {
        return (-alpha);
    }
    else if ((xs > xc) && (ys == yc)) {
        return alpha;
    }
    else if ((xs < xc) && (ys == yc)) {
        return (PI - alpha);
    }
    else {
        return alpha;
    }
}

double OffboardControl::ToEulerYaw(const Eigen::Quaterniond& q){
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

void OffboardControl::triangleFLight(std::vector<Eigen::Vector3d> local_sp, double hz) {
    ros::Rate rate(hz);
    TriangleForm plan;
    plan_init = false;
    while(ros::ok()) {
        if(!plan_init) {
            plan.setStart(mav_pos_,mav_yaw_);
            for(auto lsp : local_setpoint_)
            plan.appendSetpoint(lsp);
            plan.execute();
            plan.getWpIndex(Indexwp);
            sample_size = plan.getSize();
            plan_init = true;
        }
        if(plan_init){
        sample_idx ++;
        if(sample_idx < sample_size) { 
            if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
            // controller_msgs::PositionCommand cmd;
            cmd_.position = toGeometry_msgs(plan.getPos(sample_idx));
            cmd_.velocity = toVector3(plan.getVel(sample_idx));
            cmd_.acceleration = toVector3(plan.getAcc(sample_idx));
            cmd_.yaw = plan.getYaw(sample_idx);
            pos_cmd_.publish(cmd_);
            // geometry_msgs::Point acc_msg;
            // acc_msg = toGeometry_msgs(plan.getAcc(sample_idx));
            // acc_cmd.publish(acc_msg);
            ROS_INFO_STREAM("Sec2 next waypoint " << (Indexwp[waypoint_idx] - sample_idx)*0.01 
            << " Sec to end " << (sample_size - sample_idx) *0.01);
            } else {
                break;
                }
        }
    ros::spinOnce();
    rate.sleep();
    }
}

bool OffboardControl::checkPlanError(double error, Eigen::Vector3d x) {
    Eigen::Vector3d geo_error;
    geo_error << x(0) - current_odom_.pose.pose.position.x, x(1) - current_odom_.pose.pose.position.y, x(2) - current_odom_.pose.pose.position.z;
    return (geo_error.norm() < error) ? true : false;
}

// geometry_msgs::Point OffboardControl::local3dspConvert(Eigen::Vector3d x) {
//     geometry_msgs::Point y;
//     y.x = x(0);
//     y.y = x(1);
//     y.z = x(2);
//     return y;
// }

geometry_msgs::PoseArray OffboardControl::local3dspConvert(std::vector<Eigen::Vector3d> x) {
    geometry_msgs::PoseArray y;
    for(int i = 0; i < x.size(); i++) {
        y.poses[i].position.x = x[i](0);
        y.poses[i].position.y = x[i](1);
        y.poses[i].position.z = x[i](2);
    }
    return y;
}