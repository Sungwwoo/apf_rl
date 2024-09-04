#include "potential_fields/basic_apf.hpp"

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(potential_fields::BasicAPF)

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace potential_fields{
    BasicAPF::BasicAPF(const rclcpp::NodeOptions& options): Node("basic_apf", options){
        RCLCPP_INFO(this->get_logger(), "Launching ROS2 APF node");
        RCLCPP_INFO(this->get_logger(), "Using namespace %s", this->get_namespace());

        RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
        std::map<std::string, bool> bool_params{
            {"check_potential", false}
        };
        std::map<std::string, int> int_params{
            {"k", 10},
            {"eta", 100}};
        std::map<std::string, double> double_params{
            {"attractive_potential_gain", 10.0},
            {"repulsive_potential_gain", 300.0},
            {"attractive_rho_o", 3.0},
            {"potential_field_distance", 25.0},
            {"vel_x_max", 1.5},
            {"vel_x_min", 0.0},
            {"acc_x_lim", 3.0},
            {"linear_kp", 0.5},
            {"acc_theta_lim", 3.0},
            {"vel_theta_max", 1.5},
            {"vel_theta_min", 0.0},
            {"angular_kp", 3.0},
            {"angular_kd", 2.0},
            {"xy_goal_tolerance", 0.2},
            {"yaw_goal_tolerance", 0.1},
            {"in_place_vel_theta", 0.2},
        };
        this->declare_parameters("", bool_params);
        this->declare_parameters("", int_params);
        this->declare_parameters("", double_params);

        RCLCPP_INFO(this->get_logger(), "Loading parameters from param.yaml");
        KP_ = this->get_parameter("k").as_int();
        ETA_ = this->get_parameter("eta").as_int();        
        pf_distance_ = this->get_parameter("potential_field_distance").as_double();
        linear_kp_ = this->get_parameter("linear_kp").as_double();
        angular_kp_ = this->get_parameter("angular_kp").as_double();
        angular_kd_ = this->get_parameter("angular_kd").as_double();
        acc_x_lim_ = this->get_parameter("acc_x_lim").as_double();
        vel_x_max_ = this->get_parameter("vel_x_max").as_double();
        vel_x_min_ = this->get_parameter("vel_x_min").as_double();
        acc_theta_lim_ = this->get_parameter("acc_theta_lim").as_double();
        vel_theta_max_ = this->get_parameter("vel_theta_max").as_double();
        vel_theta_min_ = this->get_parameter("vel_theta_min").as_double();
        in_place_vel_theta_ = this->get_parameter("in_place_vel_theta").as_double();
        xy_goal_tol_ = this->get_parameter("xy_goal_tolerance").as_double();
        yaw_goal_tol_ = this->get_parameter("yaw_goal_tolerance").as_double();
        check_potential_ = this->get_parameter("check_potential").as_bool();

        first_scan_ = true;
        
        RCLCPP_INFO(this->get_logger(), "All parameters set");

        RCLCPP_INFO(this->get_logger(), "Initializing node components");
        // Set publishers and subscribers
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("potential_markers", 10);

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&BasicAPF::cb_scan, this, _1)
        );

        // Set services
        srv_get_weights_ = this->create_service<apf_interfaces::srv::GetWeights>(
                                "get_weights", 
                                std::bind(&BasicAPF::getWeights, this, _1, _2));
        srv_set_weights_ = this->create_service<apf_interfaces::srv::SetWeights>(
                                "set_weights", 
                                std::bind(&BasicAPF::setWeights, this, _1, _2));
        srv_toggle_run_ = this->create_service<apf_interfaces::srv::ToggleRun>(
                                "toggle_run",
                                std::bind(&BasicAPF::toggleRun, this, _1, _2));
        srv_get_forces_ = this->create_service<apf_interfaces::srv::GetForces>(
                                "get_forces",
                                std::bind(&BasicAPF::getForces, this, _1, _2));

        // tf listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initializing forces
        f_total_ = {0.0, 0.0, 0.0};
        f_att_ = {0.0, 0.0, 0.0};
        f_rep_ = {0.0, 0.0, 0.0};

        prev_tic_ = 0.0; prev_v_ = 0.0; prev_w_ = 0.0; prev_yaw_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Deleting existing RVIZ markers");
        delete_markers_ = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_markers_.markers.push_back(marker);
        pub_markers_->publish(delete_markers_);
        rclcpp::sleep_for(5s);
        RCLCPP_INFO(this->get_logger(), "APF Initialized");
    }

    void BasicAPF::setWeights(const std::shared_ptr<apf_interfaces::srv::SetWeights::Request> request,
                        std::shared_ptr<apf_interfaces::srv::SetWeights::Response> response){
        KP_ = request->att;
        ETA_ = request->rep;

        if (request->att == KP_ && request->rep == ETA_)
            response->result = true;
        else
            response->result = false;
    }

    void BasicAPF::getWeights(const std::shared_ptr<apf_interfaces::srv::GetWeights::Request> request,
                        std::shared_ptr<apf_interfaces::srv::GetWeights::Response> response){
        
        response->att = KP_;
        response->rep = ETA_;
    }
    void BasicAPF::toggleRun(const std::shared_ptr<apf_interfaces::srv::ToggleRun::Request> request,
                std::shared_ptr<apf_interfaces::srv::ToggleRun::Response> response){
        is_running_ = request->target_state;
        response->current_state = is_running_;
    }

    void BasicAPF::getForces(const std::shared_ptr<apf_interfaces::srv::GetForces::Request> request,
                std::shared_ptr<apf_interfaces::srv::GetForces::Response> response){
        std::vector<double> data;
        data.insert(data.end(), f_att_.begin(), f_att_.end());
        data.insert(data.end(), f_rep_.begin(), f_rep_.end());
        data.insert(data.end(), f_total_.begin(), f_total_.end());
        response->forces = data;
    }


    void BasicAPF::getGoalPose(){
        try {
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform("goal", "base_link", tf2::TimePointZero);
            goal_x_ = t.transform.translation.x;
            goal_y_ = t.transform.translation.y;
            tf2::Quaternion q(
                t.transform.rotation.x, 
                t.transform.rotation.y, 
                t.transform.rotation.z, 
                t.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double r, p;
            m.getRPY(r, p, goal_orientation_);
        }
        catch (const tf2::TransformException& ex){
            RCLCPP_INFO(this->get_logger(), "Could not find transform of goal: %s", ex.what());
        }
    }

    void BasicAPF::calcAttractiveForce(){
        double d = std::hypot(goal_x_, goal_y_);
        double ratio = 1.0;

        if (d < att_rho_o_){
            ratio = d / att_rho_o_;
            d = att_rho_o_;
        }

        double theta = std::atan2(goal_x_, goal_y_);
        theta += att_offset_;

        double x = KP_ * d * std::sin(theta) / ratio;
        double y = KP_ * d * std::cos(theta) / ratio;

        f_att_ = {x, y, std::hypot(x, y)};
    }

    void BasicAPF::calcRepulsiveForce(Point2D obstacle){
        double* rect = obstacle.getXY();
        double r = obstacle.getRTheta()[0];

        if (r > pf_distance_)
            f_rep_ = {0, 0, 0};
        else{
            double x = - ETA_ * (1 / r - 1 / pf_distance_) / std::pow(r, 2) * rect[0] / r;
            double y = - ETA_ * (1 / r - 1 / pf_distance_) / std::pow(r, 2) * rect[1] / r;
            
            f_rep_ = {x, y, std::hypot(x, y)};
        }
    }

    void BasicAPF::calcPotentialField(Point2D obstacle){
        
        auto APF_info = visualization_msgs::msg::MarkerArray();
        auto text_marker = visualization_msgs::msg::Marker();
        text_marker.header.frame_id = "map";
        text_marker.ns = "weights";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point point;
        point.x = -1.0; point.y = 0.0; point.z = 0.0;
        text_marker.pose.position = point;
        std::stringstream ss;
        ss << KP_ << ", " << ETA_;
        text_marker.text = ss.str();
        text_marker.scale.z = 0.5;
        APF_info.markers.push_back(text_marker);

        calcAttractiveForce();
        auto att_marker = visualization_msgs::msg::Marker();
        att_marker.header.frame_id = ld_link_name_;
        att_marker.ns = "attractive";
        att_marker.id = 0;
        att_marker.type = visualization_msgs::msg::Marker::ARROW;
        att_marker.action = visualization_msgs::msg::Marker::ADD;
        att_marker.scale.x = 0.1 * f_att_[2];
        att_marker.scale.y = 0.03; att_marker.scale.z = 0.03;        
        att_marker.pose.position.x = 0; att_marker.pose.position.y = 0; att_marker.pose.position.z = 0;
        att_marker.pose.orientation = getQuaternion(0, 0, f_att_[0], f_att_[1]);
        att_marker.color.r = 1.0; att_marker.color.g = 0.0; att_marker.color.b = 0.0;
        att_marker.color.a = 0.7;
        APF_info.markers.push_back(att_marker);
        
        calcRepulsiveForce(obstacle);
        auto rep_marker = visualization_msgs::msg::Marker();
        rep_marker.header.frame_id = ld_link_name_;
        rep_marker.ns = "repulsive";
        rep_marker.id = 0;
        rep_marker.type = visualization_msgs::msg::Marker::ARROW;
        rep_marker.action = visualization_msgs::msg::Marker::ADD;
        rep_marker.scale.x = 0.1 * f_rep_[2];
        rep_marker.scale.y = 0.03; rep_marker.scale.z = 0.03;
        rep_marker.pose.position.x = 0; rep_marker.pose.position.y = 0; rep_marker.pose.position.z = 0;
        rep_marker.pose.orientation = getQuaternion(0.0, 0.0, f_rep_[0], f_rep_[1]);
        rep_marker.color.r = 0.0; rep_marker.color.g = 0.0; rep_marker.color.b = 1.0;
        rep_marker.color.a = 0.7;
        APF_info.markers.push_back(rep_marker);


        // lidar frame -> base frame conversion
        calcTotalForce(f_total_, f_att_[0] + f_rep_[0], f_att_[1] + f_rep_[1]);
        auto total_marker = visualization_msgs::msg::Marker();
        total_marker.header.frame_id = "base_link";
        total_marker.ns = "total";
        total_marker.id = 0;
        total_marker.type = visualization_msgs::msg::Marker::ARROW;
        total_marker.action = visualization_msgs::msg::Marker::ADD;
        total_marker.scale.x = 0.1 * f_total_[2];
        total_marker.scale.y = 0.03; total_marker.scale.z = 0.03;
        total_marker.pose.position.x = 0; total_marker.pose.position.y = 0; total_marker.pose.position.z = 0;
        total_marker.pose.orientation = getQuaternion(0.0, 0.0, f_total_[0], f_total_[1]);
        total_marker.color.r = 0.0; total_marker.color.g = 1.0; total_marker.color.b = 0.0;
        total_marker.color.a = 0.7;
        APF_info.markers.push_back(total_marker);

        pub_markers_->publish(delete_markers_);
        pub_markers_->publish(APF_info);
        
    }

    void BasicAPF::calcVelocity(){
        geometry_msgs::msg::Twist twist;
        twist.linear.y = 0.0; twist.linear.z = 0.0;
        twist.angular.x = 0.0; twist.linear.y = 0.0;

        double current_yaw = std::atan2(f_total_[0], f_total_[1]);

        // If robot is in goal position
        if (std::hypot(goal_x_, goal_y_) < xy_goal_tol_){
            // stop the robot
            twist.linear.x = 0.0;
            // align orientation
            double goal_yaw = std::atan2(goal_x_, goal_y_);
            if (goal_yaw < -yaw_goal_tol_)
                twist.angular.z = -in_place_vel_theta_;
            else if (goal_yaw > yaw_goal_tol_)
                twist.angular.z = in_place_vel_theta_;
            else{
                RCLCPP_INFO(this->get_logger(), "Goal Arrived!");
                twist.angular.z = 0.0;
                is_running_ = false;
            }
            if (!check_potential_)
                pub_cmd_->publish(twist);
            return;
        }

        rclcpp::Time current_tic = this->get_clock()->now();

        double target_ang = vel_theta_max_ * angular_kp_ * current_yaw / (2 * 3.141592) - angular_kd_ * (prev_yaw_ - current_yaw);
        if (target_ang > vel_theta_max_)
            twist.angular.z = vel_theta_max_;
        else if (target_ang < -vel_theta_max_)
            twist.angular.z = -vel_theta_max_;
        else
            twist.angular.z = target_ang;

        double angular_ratio = 1.3 * std::abs(twist.angular.z) / vel_theta_max_;
        if (angular_ratio > 0.9)
            angular_ratio = 0.9;

        double target_lin = (1 - angular_ratio) * linear_kp_ * f_total_[2];
        if (target_lin > vel_x_max_)
            twist.linear.x = vel_x_max_;
        else if (target_lin < vel_x_min_)
            twist.linear.x = vel_x_min_;
        else
            twist.linear.x = target_lin;
        
        if (!check_potential_)
            pub_cmd_->publish(twist);
        
        return;
    }

    void BasicAPF::calcTotalForce(std::vector<double>& force, double x, double y){
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = ld_link_name_;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        try{
            geometry_msgs::msg::PoseStamped converted_pose;
            converted_pose = tf_buffer_->transform(pose_stamped, "base_link");
            double nx, ny;
            nx = converted_pose.pose.position.x; ny = converted_pose.pose.position.y;
            force = {nx, ny, std::hypot(nx, ny)};

        }
        catch (tf2::TransformException& ex){
            RCLCPP_ERROR(this->get_logger(), "Force frame conversion failed: %s", ex.what());
        }
    }

    geometry_msgs::msg::Quaternion getQuaternion(double x1, double y1, double x2, double y2){
        double dx = x2 - x1;
        double dy = y2 - y1;

        double yaw = std::atan2(dy, dx);
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);

        geometry_msgs::msg::Quaternion ret;
        ret.x = quat.x();
        ret.y = quat.y();
        ret.z = quat.z();
        ret.w = quat.w();

        return ret;
    }

    void BasicAPF::cb_scan(const sensor_msgs::msg::LaserScan& scan){
        if (first_scan_){
            ld_dist_max_ = scan.range_max;
            ld_dist_min_ = scan.range_min;
            ld_angle_max_ = scan.angle_max;
            ld_angle_min_ = scan.angle_min;
            ld_data_len_ = scan.ranges.size();
            ld_angle_step_ = scan.angle_increment;
            ld_link_name_ = scan.header.frame_id;
            first_scan_ = false;
            RCLCPP_INFO(this->get_logger(), "Initialized Lidar parameters");
        }

        if (!is_running_)
            return;
        
        // Extract distances
        std::vector<float> obs_raw = scan.ranges;

        // Find the iterator to the minimum element
        auto minDist = std::min_element(obs_raw.begin(), obs_raw.end());

        // Find the index of the minimum element
        int minIndex = std::distance(obs_raw.begin(), minDist);

        // Choose closest point
        Point2D min_dist_point;
        min_dist_point.setRTHeta(*minDist, ld_angle_min_ + ld_angle_step_ * minIndex);

        // Relative goal pose
        getGoalPose();

        // Calculate Potential Field
        calcPotentialField(min_dist_point);

        // Control
        calcVelocity();
    }

}