#ifndef BASIC_APF_HPP__
#define BASIC_APF_HPP__

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "point_2d.hpp"
#include "apf_interfaces/srv/get_weights.hpp"
#include "apf_interfaces/srv/set_weights.hpp"
#include "apf_interfaces/srv/toggle_run.hpp"
#include "apf_interfaces/srv/get_forces.hpp"

namespace potential_fields{

    class BasicAPF : public rclcpp::Node{
        public:
            BasicAPF(const rclcpp::NodeOptions& options);

        private:
            void getGoalPose();
            void calcAttractiveForce();
            void calcRepulsiveForce(Point2D obstacle);
            void calcPotentialField(Point2D);
            void calcVelocity();
            void calcTotalForce(std::vector<double>&, double, double);
            void cb_scan(const sensor_msgs::msg::LaserScan& scan);

            void setWeights(const std::shared_ptr<apf_interfaces::srv::SetWeights::Request> request,
                        std::shared_ptr<apf_interfaces::srv::SetWeights::Response> response);
            void getWeights(const std::shared_ptr<apf_interfaces::srv::GetWeights::Request> request,
                        std::shared_ptr<apf_interfaces::srv::GetWeights::Response> response);
            
            void toggleRun(const std::shared_ptr<apf_interfaces::srv::ToggleRun::Request> request,
                        std::shared_ptr<apf_interfaces::srv::ToggleRun::Response> response);
            void getForces(const std::shared_ptr<apf_interfaces::srv::GetForces::Request> request,
                        std::shared_ptr<apf_interfaces::srv::GetForces::Response> response);

            /*************  services  **************/ 
            rclcpp::Service<apf_interfaces::srv::GetWeights>::SharedPtr srv_get_weights_;
            rclcpp::Service<apf_interfaces::srv::SetWeights>::SharedPtr srv_set_weights_;
            rclcpp::Service<apf_interfaces::srv::ToggleRun>::SharedPtr srv_toggle_run_;
            rclcpp::Service<apf_interfaces::srv::GetForces>::SharedPtr srv_get_forces_;
            
            /************* publishers **************/ 
            // cmd_vel
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
            // visualizer
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

            /************* subscribers **************/
            // scan
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
            // transform listener;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            /************* Parameters **************/
            std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;

            // APF parameters
            int KP_, ETA_;
            double att_rho_o_, att_offset_;
            double ld_dist_max_, ld_dist_min_, ld_angle_max_, ld_angle_min_, ld_data_len_, ld_angle_step_;
            std::string ld_link_name_;
            double pf_distance_;
            double linear_kp_, angular_kp_, angular_kd_;

            // Robot dynamics parameters
            double acc_x_lim_, vel_x_max_, vel_x_min_, acc_theta_lim_, vel_theta_max_, vel_theta_min_;
            double in_place_vel_theta_;

            // Goal tolerance
            double xy_goal_tol_, yaw_goal_tol_;

            bool check_potential_;
            bool is_running_;
            bool first_scan_;

            double goal_x_, goal_y_, goal_orientation_;
            double prev_tic_, prev_v_, prev_w_, prev_yaw_;
            visualization_msgs::msg::MarkerArray delete_markers_;
            std::vector<double> f_total_, f_att_, f_rep_;

            geometry_msgs::msg::Quaternion getQuaternion(double, double, double, double);
    };
}

#endif
