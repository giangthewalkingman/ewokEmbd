#include "offboard/offboard.h"
// #include "geometric_controller/geometric_controller.h"
// OffboardControl::OffboardControl(){
// }

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private),
                                                                                                                      simulation_mode_enable_(false),
                                                                                                                      delivery_mode_enable_(false),
                                                                                                                      return_home_mode_enable_(false) {
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    gps_position_sub_ = nh_.subscribe("/mavros/global_position/global", 10, &OffboardControl::gpsPositionCallback, this);
    opt_point_sub_ = nh_.subscribe("optimization_point", 10, &OffboardControl::optPointCallback, this);
    point_target_sub_ = nh_.subscribe("point_target",10, &OffboardControl::targetPointCallback, this);
    check_last_opt_sub_ = nh_.subscribe("check_last_opt_point",10, &OffboardControl::checkLastOptPointCallback, this);

    //DuyNguyen
    marker_p_sub_ = nh_.subscribe("/target_pos",10, &OffboardControl::markerCallback, this);
    check_move_sub_ = nh_.subscribe("/move_position",10, &OffboardControl::checkMoveCallback, this);
    ids_detection_sub_ = nh_.subscribe("/ids_detection",10, &OffboardControl::checkIdsDetectionCallback, this);
    local_p_sub_ = nh_.subscribe("/mavros/local_position/pose",10, &OffboardControl::poseCallback, this);


    // setpoint_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pos_cmd_ = nh_.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd", 1);
    odom_error_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_error", 1, true);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd_/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    nh_private_.param<bool>("/offboard_node/simulation_mode_enable", simulation_mode_enable_, simulation_mode_enable_);
    nh_private_.param<bool>("/offboard_node/delivery_mode_enable", delivery_mode_enable_, delivery_mode_enable_);
    nh_private_.param<bool>("/offboard_node/return_home_mode_enable", return_home_mode_enable_, return_home_mode_enable_);
    nh_private_.getParam("/offboard_node/number_of_target", num_of_enu_target_);
    nh_private_.getParam("/offboard_node/target_error", target_error_);
    nh_private_.getParam("/offboard_node/target_x_pos", x_target_);
    nh_private_.getParam("/offboard_node/target_y_pos", y_target_);
    nh_private_.getParam("/offboard_node/target_z_pos", z_target_);
    //nh_private_.getParam("/offboard_node/target_yaw", yaw_target_);
    nh_private_.getParam("/offboard_node/number_of_goal", num_of_gps_goal_);
    nh_private_.getParam("/offboard_node/goal_error", goal_error_);
    nh_private_.getParam("/offboard_node/latitude", lat_goal_);
    nh_private_.getParam("/offboard_node/longitude", lon_goal_);
    nh_private_.getParam("/offboard_node/altitude", alt_goal_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh_private_.getParam("/offboard_node/z_delivery", z_delivery_);
    nh_private_.getParam("/offboard_node/land_error", land_error_);
    nh_private_.getParam("/offboard_node/takeoff_hover_time", takeoff_hover_time_);
    nh_private_.getParam("/offboard_node/hover_time", hover_time_);
    nh_private_.getParam("/offboard_node/unpack_time", unpack_time_);
    nh_private_.getParam("/offboard_node/desired_velocity", vel_desired_);
    nh_private_.getParam("/offboard_node/land_velocity", land_vel_);
    nh_private_.getParam("/offboard_node/return_velcity", return_vel_);

    nh_private_.getParam("/offboard_node/yaw_rate", yaw_rate_);
    // nh_private_.getParam("/offboard_node/yaw_error", yaw_error_);
    nh_private_.getParam("/offboard_node/odom_error", odom_error_);

    waitForPredicate(10.0);
    if (input_setpoint) {
        inputSetpoint();
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
    if (simulation_mode_enable_) {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation_mode_enable' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=false\n");
    }
    else {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation_mode_enable' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=true\n");
    }
    operation_time_1_ = ros::Time::now();
}

/* send a few setpoints before publish
   input: ros rate in hertz (at least 2Hz) and first setpoint */
void OffboardControl::setOffboardStream(double hz, geometry_msgs::PoseStamped first_target) {
    ros::Rate rate(hz);
    std::printf("[ INFO] Setting OFFBOARD stream \n");
    for (int i = 50; ros::ok() && i > 0; --i) {
        target_enu_pose_ = first_target;
        // std::printf("\n[ INFO] first_target ENU position: [%.1f, %.1f, %.1f]\n", target_enu_pose_.pose.position.x, target_enu_pose_.pose.position.y, target_enu_pose_.pose.position.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        // std::printf("\n[ INFO] second ENU position: [%.1f, %.1f, %.1f]\n", target_enu_pose_.pose.position.x, target_enu_pose_.pose.position.y, target_enu_pose_.pose.position.z);
        // setpoint_pose_pub_.publish(target_enu_pose_);
        cmd_.position.x = first_target.pose.position.x;
        cmd_.position.y = first_target.pose.position.y;
        cmd_.position.z = first_target.pose.position.z;
        pos_cmd_.publish(cmd_);
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] OFFBOARD stream is set\n");
}

/* wait for ARM and OFFBOARD mode switch (in SITL case or HITL/Practical case)
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForArmAndOffboard(double hz) {
    ros::Rate rate(hz);
    if (simulation_mode_enable_) {
        std::printf("\n[ INFO] Ready to takeoff\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            mavros_msgs::CommandBool arm_amd;
            arm_amd.request.value = true;
            if (arming_client_.call(arm_amd) && arm_amd.response.success) {
                ROS_INFO_ONCE("Vehicle armed");
            }
            else {
                ROS_INFO_ONCE("Arming failed");
            }

            // mavros_msgs::SetMode offboard_setmode_;
            offboard_setmode_.request.base_mode = 0;
            offboard_setmode_.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(offboard_setmode_) && offboard_setmode_.response.mode_sent) {
                ROS_INFO_ONCE("OFFBOARD enabled");
            }
            else {
                ROS_INFO_ONCE("Failed to set OFFBOARD");
            }
            ros::spinOnce();
            rate.sleep();
        }
        //DuyNguyen
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
    else {
        std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            ros::spinOnce();
            rate.sleep();
        }
        //DuyNguyen
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
}

/* wait drone get a stable state
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForStable(double hz) {
    ros::Rate rate(hz);
    std::printf("\n[ INFO] Waiting for stable state\n");

    ref_gps_position_ = current_gps_position_;
    geometry_msgs::Point converted_enu;
    for (int i = 0; i < 100; i++) {
        converted_enu = WGS84ToENU(current_gps_position_, ref_gps_position_);
        x_off_[i] = current_odom_.pose.pose.position.x - converted_enu.x;
        y_off_[i] = current_odom_.pose.pose.position.y - converted_enu.y;
        z_off_[i] = current_odom_.pose.pose.position.z - converted_enu.z;
        ros::spinOnce();
        rate.sleep();
    }
    for (int i = 0; i < 100; i++) {
        x_offset_ = x_offset_ + x_off_[i] / 100;
        y_offset_ = y_offset_ + y_off_[i] / 100;
        z_offset_ = z_offset_ + z_off_[i] / 100;
    }
    std::printf("[ INFO] Got stable state\n");

    home_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z, yaw_);
    home_gps_position_ = current_gps_position_;
    std::printf("\n[ INFO] Got HOME position: [%.1f, %.1f, %.1f, %.1f]\n", home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z, tf::getYaw(home_enu_pose_.pose.orientation));
    std::printf("        latitude : %.8f\n", home_gps_position_.latitude);
    std::printf("        longitude: %.8f\n", home_gps_position_.longitude);
    std::printf("        altitude : %.8f\n", home_gps_position_.altitude);
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom_ = *msg;
    odom_received_ = true;
    tf::poseMsgToEigen(current_odom_.pose.pose, current_pose_);
    tf::vectorMsgToEigen(current_odom_.twist.twist.linear, current_velocity_);
    yaw_ = tf::getYaw(current_odom_.pose.pose.orientation);
    mav_pos_ = toEigen(current_odom_.pose.pose.position);
    // std::cout << "\n[Debug] yaw from odom: " << degreeOf(yaw_) << "\n";
}

void OffboardControl::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps_position_ = *msg;
    gps_received_ = true;
}

void OffboardControl::optPointCallback(const geometry_msgs::Point::ConstPtr &msg) {
    // opt_point_ = *msg;
    // opt_point_ = *msg;
    opt_point_ = *msg;
    //optimization_point_.push_back(*msg);
    opt_point_received_ = true;
    ROS_INFO_STREAM("Hello World");
}

void OffboardControl::targetPointCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    target_array_ = *msg;
}

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

/* manage input: select mode, setpoint type, ... */
void OffboardControl::inputSetpoint() {
    // std::printf("\n[ INFO] Please choose (1) to enter mode: Planning and Marker\n");
    // char mode;
    // std::printf("(1): ");
    // std::cin >> mode;
    // if (mode == '1') {
        std::printf("\n[ INFO] Mode 1: Mission with planner and marker\n");
        std::printf("[ INFO] Parameter 'return_home_mode_enable' is set %s\n", return_home_mode_enable_ ? "true" : "false");
        std::printf("[ INFO] Parameter 'delivery_mode_enable' is set %s\n", delivery_mode_enable_ ? "true" : "false");
        inputPlannerAndLanding();
    // }
    // else {
    //     // std::printf("\n[ WARN] Not avaible mode\n");
    //     // inputSetpoint();
    //     ros::shutdown();
    // }
}

void OffboardControl::inputPlannerAndLanding() {
    waitForStable(10.0);
    setOffboardStream(10.0, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    waitForArmAndOffboard(10.0);
    takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), takeoff_hover_time_);
    int count = end(target_array_.data) - begin(target_array_.data);
    num_of_enu_target_ = (count)/3;
    global_point_(0) = target_array_.data[0];
    global_point_(1) = target_array_.data[1];
    global_point_(2) = target_array_.data[2];
    local_setpoint_.push_back(global_point_);
    ros::Rate rate(10.0);
    std::printf("[ INFO] Loaded global path setpoints: \n");
    // rate.sleep();
    for(int i = 0; i < num_of_enu_target_; i++) {
        std::cout << "Setpoint " << i+1 << ": " << target_array_.data[i*3+0] << ", " << target_array_.data[i*3+1] << ", " << target_array_.data[i*3+2] << ", " << std::endl;
    }
    std::printf("[ INFO] Loaded the start setpoint: \n");
    for(auto lsp: local_setpoint_){
    ROS_INFO_STREAM("Start sp: " << lsp(0) << " " << lsp(1) << " " << lsp(2));
    }
    std::printf("\n[ INFO] Flight with Planner setpoint\n");
    std::printf("\n[ INFO] Flight to the start point of Optimization path\n");
    plannerAndLandingFlight();
}

void OffboardControl::plannerAndLandingFlight() {
    bool first_target_reached = false;
    bool stop = false;
    bool final_reached = false;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(100);
    //double curr_alpha_hover;
    bool first_receive_hover = true;

    double target_alpha,this_loop_alpha;
    //work in progress
    //point to hold position when yaw angle is to high. Save this position and publish this position with yaw when need to rotate high yaw angle will help drone hold position. Update this position constantly when moving
    double current_hold_x = current_odom_.pose.pose.position.x;
    double current_hold_y = current_odom_.pose.pose.position.y;
    double current_hold_z = current_odom_.pose.pose.position.z;
    controller_msgs::PositionCommand cmd;
        cmd.position.x = global_point_(0);
        cmd.position.y = global_point_(1);
        cmd.position.z = global_point_(2);

    while (ros::ok()) {
        if(!plan_init){
       plan.setStart(mav_pos_,yaw_);
       for(auto lsp : local_setpoint_)
       plan.appendSetpoint(lsp);
       plan.execute();
       plan.getWpIndex(Indexwp);
       sample_size = plan.getSize();
       plan_init = true;
       }
       if(plan_init){
            sample_idx ++;
            if(sample_idx < sample_size)
            {  
                if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
                cmd_.position = toGeometry_msgs(plan.getPos(sample_idx));
                cmd_.velocity = toVector3(plan.getVel(sample_idx));
                cmd_.acceleration = toVector3(plan.getAcc(sample_idx));
                cmd_.yaw = plan.getYaw(sample_idx);
                pos_cmd_.publish(cmd_);
                // ROS_INFO_STREAM("Sec2 next waypoint " << (Indexwp[waypoint_idx] - sample_idx)*0.01 
                // << " Sec to end " << (sample_size - sample_idx) *0.01);
            }
            // else{
            //     break;
            //     }
        }
        first_target_reached = checkPosCmdError(target_error_, cmd);
        distance_ = distancePosCmdBetween(cmd);
        std::printf("Distance to target: %.1f (m) \n", distance_);

        if (distance_ <= 0.3) {
            std::printf("\n[ INFO] Reached start point of Optimization path\n");
            // hovering(targetTransfer(global_point_(0), global_point_(1), global_point_(2)), 2.0);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    hovering(targetTransfer(global_point_(0), global_point_(1), global_point_(2)), 10);
    opt_point_received_ = true;
    yaw_rate_ = 0.15;
    first_target_reached = false;
    ros::Time current_time = ros::Time::now();
    if (opt_point_received_) {
        std::printf("\n[ INFO] Fly with optimization points\n");  
        bool first_receive = true;
        double target_alpha, this_loop_alpha;
        bool flag = true;
        plan_init = false;
        // plan.setStart(mav_pos_,yaw_);
        bool continue_to_append = true;
        Eigen::Vector3d opt_3d_point_;
        while (ros::ok() && !stop) {
             std::cout << "cmd_: " << cmd_.position.x << ", " << cmd_.position.y << ", " << cmd_.position.z << ", " << std::endl;
             std::cout << "opt_point_: " << opt_point_.x << ", " << opt_point_.y << ", " << opt_point_.z << ", " << std::endl;
             std::cout << "opt_3d_point: " << opt_3d_point_(0) << ", " << opt_3d_point_(1) << ", " << opt_3d_point_(2) << ", " << std::endl;
                        // std::cout << "opt_point_: " << opt_point_.x << ", " << opt_point_.y << ", " << opt_point_.z << ", " << std::endl;
            // if(continue_to_append){
            // if(1){
            //     // Eigen::Vector3d opt_3d_point_;
            //     opt_3d_point_(0) = opt_point_.x;
            //     opt_3d_point_(1) = opt_point_.y;
            //     opt_3d_point_(2) = opt_point_.z;
            //     plan.setStart(mav_pos_,yaw_);
            //     // for(auto lsp : local_setpoint_)
            //     plan.appendSetpoint(opt_3d_point_);
            //     plan.execute();
            //     plan.getWpIndex(Indexwp);
            //     sample_size = plan.getSize();
            //     plan_init = true;
            //     continue_to_append = false;
            // }
            //     if(plan_init){
            //         sample_idx ++;
            //         if(sample_idx < sample_size)
            //         {  
            //             if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
            //             cmd_.position = toGeometry_msgs(plan.getPos(sample_idx));
            //             cmd_.velocity = toVector3(plan.getVel(sample_idx));
            //             cmd_.acceleration = toVector3(plan.getAcc(sample_idx));
            //             cmd_.yaw = plan.getYaw(sample_idx);
            //             pos_cmd_.publish(cmd_);
            //             // std::cout << "cmd_: " << cmd_.position.x << ", " << cmd_.position.y << ", " << cmd_.position.z << ", " << std::endl;
            //             // std::cout << "opt_point_: " << opt_point_.x << ", " << opt_point_.y << ", " << opt_point_.z << ", " << std::endl;
            //             // continue_to_append = checkPosCmdError(target_error_, opt_point_);
            //         }
            //         else{
            //         //  break;
            //         }
            //     }
            cmd_.position = opt_point_;
            pos_cmd_.publish(cmd_);
            final_reached = checkPositionError(target_error_, targetTransfer(target_array_.data[3], target_array_.data[4], target_array_.data[5]));

            if (final_reached && check_last_opt_point_.data) {
                std::printf("\n[ INFO] Reached Final position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
                //hovering(setpoint, hover_time_);
                hovering(targetTransfer(target_array_.data[3], target_array_.data[4], target_array_.data[5], degreeOf(yaw_)), hover_time_);

                // DuyNguyen => Start landing
                // first case: when uav arrives to destination but it do not have marker.
                // hover and wait id_marker after 5 seconds uav will auto land
                ros::spinOnce();                                                                                                                         
                rate.sleep();
                if (check_ids_ == false) {
                    std::printf("\n[ INFO] No ids and Landing\n");
                    if (!return_home_mode_enable_) {
                        //landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
                        landingYaw(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0, degreeOf(yaw_)));
                    }
                    else {
                        if (delivery_mode_enable_) {
                            delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
                        }
                        std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n", home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                        returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                        landing(home_enu_pose_);
                    }
                    break;
                }
                else {
                    // std::printf("\n[ INFO] Duy Landing marker\n");
                    current_z_ = setpoint.pose.position.z;
                    flag = true;
                    while (flag && ros::ok()) {
                        ros::spinOnce();
                        rate.sleep();
                        if (check_ids_ == true) {
                            if (check_mov_ == true) {
                                setpoint.pose.position.x = marker_position_.pose.position.x;
                                setpoint.pose.position.y = marker_position_.pose.position.y;
                                setpoint.pose.position.z = current_z_;
                                std::printf("\n[ INFO] Fly to marker position [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
                                
                                while (!first_target_reached && ros::ok()) {
                                    setpoint = targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);

                                    components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
                                    target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x+components_vel_.x, current_odom_.pose.pose.position.y+components_vel_.y, current_odom_.pose.pose.position.z+components_vel_.z);
                                    
                                    target_enu_pose_.header.stamp = ros::Time::now();
                                    // setpoint_pose_pub_.publish(target_enu_pose_);
                                    // setpoint_pose_pub_.publish(setpoint);
                                    // std::printf("\n[ INFO] Position of setpoint_pose_pub_ [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
                                    // std::printf("\n[ INFO] Position of current_odom [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
                                    first_target_reached = checkPositionError(target_error_, setpoint);
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                                first_target_reached = false;
                                // hover above marker in 2 seconds
                                std::printf("\n[ INFO] Hovering above marker\n");
                                ros::Time t_check;
                                t_check = ros::Time::now();
                                while ((ros::Time::now()-t_check)<ros::Duration(2)) {   
                                    setpoint_pose_pub_.publish(setpoint);
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                                ros::spinOnce();
                                rate.sleep();
                                if (current_z_ <= 1.5) {
                                    flag = false;
                                }
                            }
                            else {
                                setpoint.pose.position.x = current_position_.pose.position.x;
                                setpoint.pose.position.y = current_position_.pose.position.y;
                                setpoint.pose.position.z = current_z_ - 1.0;
                                // update the altitude
                                current_z_ = setpoint.pose.position.z;
                                std::printf("\n[ INFO] Decrease to position [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
                                while (!first_target_reached && ros::ok()) {
                                    setpoint = targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);

                                    components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
                                    target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x+components_vel_.x, current_odom_.pose.pose.position.y+components_vel_.y, current_odom_.pose.pose.position.z+components_vel_.z);
                                    
                                    target_enu_pose_.header.stamp = ros::Time::now();
                                    setpoint_pose_pub_.publish(target_enu_pose_);
                                    // std::printf("\n[ INFO] Position of setpoint_pose_pub_ [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
                                    // std::printf("\n[ INFO] Position of current_odom [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
                                    first_target_reached = checkPositionError(target_error_, setpoint);
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                                first_target_reached = false;
                                // hover above marker in 2 seconds
                                std::printf("\n[ INFO] Hover above marker\n");
                                ros::Time t_check;
                                t_check = ros::Time::now();
                                while ((ros::Time::now()-t_check)<ros::Duration(2)) {   
                                    setpoint_pose_pub_.publish(setpoint);
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                                ros::spinOnce();
                                rate.sleep();
                                if (current_z_ <= 1.5) {
                                    flag = false;
                                }
                            }
                        }
                        else {
                            std::printf("\n[ INFO] No ids and hover\n");
                            ros::Time t_check;
                            t_check = ros::Time::now();
                            while ((ros::Time::now()-t_check)<ros::Duration(5)) {   
                                setpoint_pose_pub_.publish(setpoint);
                                ros::spinOnce();
                                rate.sleep();
                            }
                            flag = false;
                        } 
                    }
                }      
            }
            ros::spinOnce();
            rate.sleep();
            if (flag == false) {
                stop = true;
            }
            //stop = true;
        }
        offboard_setmode_.request.custom_mode = "AUTO.LAND";
        if( set_mode_client_.call(offboard_setmode_) && offboard_setmode_.response.mode_sent) {
            ROS_INFO("\n[ INFO] --------------- LAND ---------------\n");
        }
    }
    else {
        std::printf("\n[ WARN] Not received optimization points! Landing\n");
        landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
    }
}

/* convert from WGS84 GPS (LLA) to ENU x,y,z
   input: GPS in WGS84 and reference GPS */
geometry_msgs::Point OffboardControl::WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref) {
    geometry_msgs::Point ecef = WGS84ToECEF(wgs84);
    geometry_msgs::Point enu = ECEFToENU(ecef, ref);
    return enu;
}

/* convert from WGS84 GPS (LLA) to ECEF x,y,z
   input: GPS (LLA) in WGS84 (sensor_msgs::NavSatFix) */
geometry_msgs::Point OffboardControl::WGS84ToECEF(sensor_msgs::NavSatFix wgs84) {
    geometry_msgs::Point ecef;
    double lambda = radianOf(wgs84.latitude);
    double phi = radianOf(wgs84.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    ecef.x = (wgs84.altitude + N) * cos_lambda * cos_phi;
    ecef.y = (wgs84.altitude + N) * cos_lambda * sin_phi;
    ecef.z = (wgs84.altitude + (1 - e_sq) * N) * sin_lambda;

    return ecef;
}

/* convert from ECEF x,y,z to ENU x,y,z
   input: point in ECEF and reference GPS */
geometry_msgs::Point OffboardControl::ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref) {
    geometry_msgs::Point enu;
    double lambda = radianOf(ref.latitude);
    double phi = radianOf(ref.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (ref.altitude + N) * cos_lambda * cos_phi;
    double y0 = (ref.altitude + N) * cos_lambda * sin_phi;
    double z0 = (ref.altitude + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = ecef.x - x0;
    yd = ecef.y - y0;
    zd = ecef.z - z0;

    enu.x = -sin_phi * xd + cos_phi * yd;
    enu.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    enu.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return enu;
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

/* perform takeoff task
   input: setpoint to takeoff and hover time */
void OffboardControl::takeOff(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    std::printf("\n[ INFO] Takeoff to [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
    bool takeoff_reached = false;
    while (ros::ok() && !takeoff_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        // setpoint_pose_pub_.publish(target_enu_pose_);
        cmd_.position.x = target_enu_pose_.pose.position.x;
        cmd_.position.y = target_enu_pose_.pose.position.y;
        cmd_.position.z = target_enu_pose_.pose.position.z;
        pos_cmd_.publish(cmd_);

        takeoff_reached = checkPositionError(target_error_, setpoint);
        if (takeoff_reached) {
            hovering(setpoint, hover_time);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
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
        cmd_.yaw = tf::getYaw(setpoint.pose.orientation);
        pos_cmd_.publish(cmd_);
        // setpoint_pose_pub_.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }
}

/* perform land task
   input: set point to land (e.g., [x, y, 0.0]) */
void OffboardControl::landing(geometry_msgs::PoseStamped setpoint) {
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Landing\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        // target_enu_pose_.pose.orientation = setpoint.pose.orientation;
        setpoint_pose_pub_.publish(target_enu_pose_);

        land_reached = checkPositionError(land_error_, setpoint);

        if (current_state_.system_status == 3) {
            std::printf("\n[ INFO] Land detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                break;
            }
        }
        else if (land_reached) {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }

    operation_time_2_ = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2_ - operation_time_1_).toSec());
    ros::shutdown();
}

void OffboardControl::landingYaw(geometry_msgs::PoseStamped setpoint) {
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Landing\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z, setpoint.pose.orientation);
        target_enu_pose_.header.stamp = ros::Time::now();
        // target_enu_pose_.pose.orientation = setpoint.pose.orientation;
        // setpoint_pose_pub_.publish(target_enu_pose_);
        cmd_.position.x = target_enu_pose_.pose.position.x;
        cmd_.position.y = target_enu_pose_.pose.position.y;
        cmd_.position.z = target_enu_pose_.pose.position.z;
        pos_cmd_.publish(cmd_);

        land_reached = checkPositionError(land_error_, setpoint);

        if (current_state_.system_status == 3) {
            std::printf("\n[ INFO] Land detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                break;
            }
        }
        else if (land_reached) {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }

    operation_time_2_ = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2_ - operation_time_1_).toSec());
    ros::shutdown();
}

/* calculate components of velocity about x, y, z axis
   input: desired velocity, current and target poses (ENU) */
geometry_msgs::Vector3 OffboardControl::velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target) {
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(sqr(dx) + sqr(dy) + sqr(dz));

    geometry_msgs::Vector3 vel;

    vel.x = ((dx / d) * v_desired);
    vel.y = ((dy / d) * v_desired);
    vel.z = ((dz / d) * v_desired);

    return vel;
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

/* perform delivery task
   input: current setpoint in trajectory and time to unpack */
void OffboardControl::delivery(geometry_msgs::PoseStamped setpoint, double unpack_time) {
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Land for unpacking\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        if (current_state_.system_status == 3) {
            land_reached = true;
        }
        else {
            land_reached = checkPositionError(land_error_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));
        }

        if (land_reached) {
            if (current_state_.system_status == 3) {
                hovering(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), unpack_time);
                // TODO: unpack service
            }
            else {
                hovering(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time);
                // TODO: unpack service
            }
            std::printf("\n[ INFO] Done! Return setpoint [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
            returnHome(setpoint);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* perform return home task
   input: home pose in ENU (e.g., [home x, home y, 10.0])*/
void OffboardControl::returnHome(geometry_msgs::PoseStamped home_pose) {
    ros::Rate rate(10.0);
    bool home_reached = false;
    while (ros::ok() && !home_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), home_pose);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        home_reached = checkPositionError(target_error_, home_pose);
        if (home_reached) {
            hovering(home_pose, hover_time_);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

