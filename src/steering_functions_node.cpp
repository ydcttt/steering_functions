/*********************************************************************
 *  Copyright (c) 2017 - for information on the respective copyright
 *  owner see the NOTICE file and/or the repository
 *
 *      https://github.com/hbanzhaf/steering_functions.git
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  permissions and limitations under the License.
 ***********************************************************************/

#include <iostream>

#include <Eigen/Dense>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include "steering_functions/dubins_state_space/dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc00_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc0pm_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/ccpm0_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/ccpmpm_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc00_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc0pm_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hcpm0_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hcpmpm_reeds_shepp_state_space.hpp"
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"
#include "steering_functions/steering_functions.hpp"

#define FRAME_ID "/world"
#define DISCRETIZATION 0.1              // [m]
#define VISUALIZATION_DURATION 2        // [s]
#define ANIMATE true                    // [-]
#define OPERATING_REGION_X 20.0         // [m]
#define OPERATING_REGION_Y 20.0         // [m]
#define OPERATING_REGION_THETA 2 * M_PI // [rad]
#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)

using namespace std;
using namespace steering;

class PathClass
{
public:
  // publisher
  ros::Publisher pub_path_;
  ros::Publisher pub_poses_; // start goal
  ros::Publisher pub_text_;

  // path properties
  string id_;
  string path_type_;
  double discretization_;
  State_With_Covariance state_start_;
  State state_goal_;
  double kappa_max_;
  double sigma_max_;
  vector<State_With_Covariance> path_;

  // filter parameters
  Motion_Noise motion_noise_;
  Measurement_Noise measurement_noise_;
  Controller controller_;

  // visualization
  string frame_id_;
  nav_msgs::Path nav_path_;
  geometry_msgs::PoseArray pose_array_;
  visualization_msgs::MarkerArray marker_array_text_;
  visualization_msgs::Marker marker_text_;

  // constructor
  PathClass(const string &path_type, const State_With_Covariance &state_start,
            const State &state_goal, const double kappa_max,
            const double sigma_max)
      : path_type_(path_type),
        discretization_(DISCRETIZATION),
        state_start_(state_start),
        state_goal_(state_goal),
        kappa_max_(kappa_max),
        sigma_max_(sigma_max),
        frame_id_(FRAME_ID)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // filter parameters
    nh.param<double>("motion_noise/alpha1", motion_noise_.alpha1, 0.1);
    nh.param<double>("motion_noise/alpha2", motion_noise_.alpha2, 0.0);
    nh.param<double>("motion_noise/alpha3", motion_noise_.alpha3, 0.0);
    nh.param<double>("motion_noise/alpha4", motion_noise_.alpha4, 0.1);

    nh.param<double>("measurement_noise/std_x", measurement_noise_.std_x, 0.1);
    nh.param<double>("measurement_noise/std_y", measurement_noise_.std_y, 0.1);
    nh.param<double>("measurement_noise/std_theta",
                     measurement_noise_.std_theta, 0.01);

    nh.param<double>("controller/k1", controller_.k1, 1.0);
    nh.param<double>("controller/k2", controller_.k2, 1.0);
    nh.param<double>("controller/k3", controller_.k3, 1.0);

    // publisher
    pub_path_ = pnh.advertise<nav_msgs::Path>("visualization_path", 10);
    pub_poses_ =
        pnh.advertise<geometry_msgs::PoseArray>("visualization_poses", 10);
    pub_text_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_text", 10);

    while (pub_path_.getNumSubscribers() == 0 ||
           pub_poses_.getNumSubscribers() == 0 ||
           pub_text_.getNumSubscribers() == 0)
      ros::Duration(0.001).sleep();

    // path
    if (path_type_ == "CC_Dubins")
    {
      id_ = "1";
      CC_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_,
                                        true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "CC00_Dubins")
    {
      id_ = "2";
      CC00_Dubins_State_Space state_space(kappa_max_, sigma_max_,
                                          discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "CC0pm_Dubins")
    {
      id_ = "3";
      CC0pm_Dubins_State_Space state_space(kappa_max_, sigma_max_,
                                           discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "CCpm0_Dubins")
    {
      id_ = "4";
      CCpm0_Dubins_State_Space state_space(kappa_max_, sigma_max_,
                                           discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "CCpmpm_Dubins")
    {
      id_ = "5";
      CCpmpm_Dubins_State_Space state_space(kappa_max_, sigma_max_,
                                            discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "Dubins")
    {
      id_ = "6";
      Dubins_State_Space state_space(kappa_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "CC00_RS")
    {
      id_ = "7";
      CC00_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                               discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "HC_RS")
    {
      id_ = "8";
      HC_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                             discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "HC00_RS")
    {
      id_ = "9";
      HC00_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                               discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "HC0pm_RS")
    {
      id_ = "10";
      HC0pm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                                discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "HCpm0_RS")
    {
      id_ = "11";
      HCpm0_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                                discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "HCpmpm_RS")
    {
      id_ = "12";
      HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_,
                                                 discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }
    else if (path_type_ == "RS")
    {
      id_ = "13";
      Reeds_Shepp_State_Space state_space(kappa_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_,
                                        controller_);
      path_ = state_space.get_path_with_covariance(state_start_, state_goal_);
    }

    // nav_path
    nav_path_.header.frame_id = frame_id_;

    // pose_array
    pose_array_.header.frame_id = frame_id_;

    // marker_text
    marker_text_.header.frame_id = frame_id_;
    marker_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text_.action = visualization_msgs::Marker::ADD;
    marker_text_.color.r = 1.0;
    marker_text_.color.g = 1.0;
    marker_text_.color.b = 1.0;
    marker_text_.color.a = 1.0;
  }

  void visualize()
  {
    // start
    marker_text_.id = 1;
    marker_text_.pose.position.x = state_start_.state.x;
    marker_text_.pose.position.y = state_start_.state.y;
    marker_text_.scale.z = 0.7;
    marker_text_.text = "start";
    marker_array_text_.markers.push_back(marker_text_);

    geometry_msgs::Pose pose_start;
    pose_start.position.x = state_start_.state.x;
    pose_start.position.y = state_start_.state.y;
    pose_start.orientation.z = sin(state_start_.state.theta / 2.0);
    pose_start.orientation.w = cos(state_start_.state.theta / 2.0);
    pose_array_.poses.push_back(pose_start);

    // goal
    marker_text_.id = 2;
    marker_text_.pose.position.x = state_goal_.x;
    marker_text_.pose.position.y = state_goal_.y;
    marker_text_.scale.z = 0.7;
    marker_text_.text = "goal";
    marker_array_text_.markers.push_back(marker_text_);

    geometry_msgs::Pose pose_goal;
    pose_goal.position.x = state_goal_.x;
    pose_goal.position.y = state_goal_.y;
    pose_goal.orientation.z = sin(state_goal_.theta / 2.0);
    pose_goal.orientation.w = cos(state_goal_.theta / 2.0);
    pose_array_.poses.push_back(pose_goal);

    // path description
    marker_text_.id = 3;
    marker_text_.pose.position.x = 0.0;
    marker_text_.pose.position.y = 12.0;
    marker_text_.scale.z = 1.0;
    marker_text_.text = id_ + ") " + path_type_ + " Steer";
    marker_array_text_.markers.push_back(marker_text_);

    // path
    for (const auto &state : path_)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = state.state.x;
      pose.pose.position.y = state.state.y;
      pose.pose.orientation.z = sin(state.state.theta / 2.0);
      pose.pose.orientation.w = cos(state.state.theta / 2.0);
      nav_path_.poses.push_back(pose);
    }

    // publish
    pub_path_.publish(nav_path_);
    pub_poses_.publish(pose_array_);
    pub_text_.publish(marker_array_text_);
    ros::spinOnce();
  }
};

class RobotClass
{
public:
  // publisher
  ros::Publisher pub_swath_;
  ros::Publisher pub_covariances_;
  ros::Publisher pub_polyhedrons_;
  ros::Publisher pub_beliefprints_;
  ros::Publisher pub_convhulls_;

  // robot config
  double kappa_max_;
  double sigma_max_;
  double wheel_base_;
  double track_width_;
  geometry_msgs::Point wheel_fl_pos_, wheel_fr_pos_;
  vector<geometry_msgs::Point> footprint_;
  vector<geometry_msgs::Point> wheel_;
  double wheel_radius_;
  double wheel_width_;

  // measurement noise
  Measurement_Noise measurement_noise_;

  // icosahedron vertices
  const double tau = 1.618033988749895; // sqrt(5)+1 / 2.0
  const double kMathEpsilon = 1e-10;
  int belief_cnt = 0;

  // visualization
  string frame_id_;
  bool animate_;
  visualization_msgs::MarkerArray marker_array_swath_;
  visualization_msgs::MarkerArray marker_array_covariance_;
  visualization_msgs::MarkerArray marker_array_polyhedron_;
  visualization_msgs::MarkerArray marker_array_beliefprint_;
  visualization_msgs::MarkerArray marker_array_convhull_;

  visualization_msgs::Marker marker_chassis_, marker_wheels_;
  visualization_msgs::Marker marker_covariance_;
  visualization_msgs::Marker marker_polyhedron_;
  visualization_msgs::Marker marker_beliefprint_;
  visualization_msgs::Marker marker_convhull_;

  // constructor
  explicit RobotClass() : frame_id_(FRAME_ID), animate_(ANIMATE)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // publisher
    pub_swath_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_swath", 10);
    pub_covariances_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_covariances", 10);
    pub_polyhedrons_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_polyhedrons", 10);
    pub_beliefprints_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_beliefprints", 10);
    pub_convhulls_ = pnh.advertise<visualization_msgs::MarkerArray>(
        "visualization_convhulls", 10);
    while (pub_swath_.getNumSubscribers() == 0 ||
           pub_covariances_.getNumSubscribers() == 0 ||
           pub_polyhedrons_.getNumSubscribers() == 0 ||
           pub_beliefprints_.getNumSubscribers() == 0 || pub_convhulls_.getNumSubscribers() == 0)
      ros::Duration(0.001).sleep();

    // robot kinematic
    nh.param<double>("kappa_max", kappa_max_, 1.0);
    nh.param<double>("sigma_max", sigma_max_, 1.0);

    // footprint
    footprint_ = costmap_2d::makeFootprintFromParams(nh);

    // wheel
    nh.getParam("wheel_base", wheel_base_);
    nh.getParam("track_width", track_width_);
    wheel_fl_pos_.x = wheel_base_;
    wheel_fl_pos_.y = track_width_ / 2.0;
    wheel_fr_pos_.x = wheel_base_;
    wheel_fr_pos_.y = -track_width_ / 2.0;

    nh.getParam("wheel_radius", wheel_radius_);
    nh.getParam("wheel_width", wheel_width_);
    geometry_msgs::Point point1, point2, point3, point4;
    point1.x = wheel_radius_;
    point1.y = wheel_width_ / 2.0;
    point2.x = -wheel_radius_;
    point2.y = wheel_width_ / 2.0;
    point3.x = -wheel_radius_;
    point3.y = -wheel_width_ / 2.0;
    point4.x = wheel_radius_;
    point4.y = -wheel_width_ / 2.0;
    wheel_.push_back(point1);
    wheel_.push_back(point2);
    wheel_.push_back(point3);
    wheel_.push_back(point4);

    // measurement noise
    nh.param<double>("measurement_noise/std_x", measurement_noise_.std_x, 0.1);
    nh.param<double>("measurement_noise/std_y", measurement_noise_.std_y, 0.1);
    nh.param<double>("measurement_noise/std_theta",
                     measurement_noise_.std_theta, 0.01);

    // marker chassis
    marker_chassis_.header.frame_id = frame_id_;
    marker_chassis_.action = visualization_msgs::Marker::ADD;
    marker_chassis_.id = 1;
    marker_chassis_.type = visualization_msgs::Marker::LINE_LIST;
    marker_chassis_.scale.x = 0.06;
    // marker_chassis_.color.r = 0.6;
    // marker_chassis_.color.g = 0.6;
    // marker_chassis_.color.b = 0.6;
    marker_chassis_.color.r = 1.0;
    marker_chassis_.color.g = 1.0;
    marker_chassis_.color.b = 1.0;
    marker_chassis_.color.a = 0.5;

    // marker wheels
    marker_wheels_.header.frame_id = frame_id_;
    marker_wheels_.action = visualization_msgs::Marker::ADD;
    marker_wheels_.id = 2;
    marker_wheels_.type = visualization_msgs::Marker::LINE_LIST;
    marker_wheels_.scale.x = 0.03;
    marker_wheels_.color.r = 0.9;
    marker_wheels_.color.g = 0.9;
    marker_wheels_.color.b = 0.9;
    marker_wheels_.color.a = 1.0;

    // marker_covariance
    marker_covariance_.header.frame_id = frame_id_;
    marker_covariance_.type = visualization_msgs::Marker::SPHERE;
    marker_covariance_.action = visualization_msgs::Marker::ADD;
    marker_covariance_.lifetime = ros::Duration(VISUALIZATION_DURATION);
    marker_covariance_.color.r = 0.6;
    marker_covariance_.color.g = 0.0;
    marker_covariance_.color.b = 0.6;
    marker_covariance_.color.a = 0.4;

    // marker polyhedron
    marker_polyhedron_.header.frame_id = frame_id_;
    marker_polyhedron_.type = visualization_msgs::Marker::LINE_LIST;
    marker_polyhedron_.action = visualization_msgs::Marker::ADD;
    marker_polyhedron_.scale.x = 0.01;
    marker_polyhedron_.color.r = 1.0;
    marker_polyhedron_.color.g = 1.0;
    marker_polyhedron_.color.b = 0.0;
    marker_polyhedron_.color.a = 0.4;

    // marker beliefprint
    marker_beliefprint_.header.frame_id = frame_id_;
    marker_beliefprint_.action = visualization_msgs::Marker::ADD;
    marker_beliefprint_.type = visualization_msgs::Marker::LINE_LIST;
    marker_beliefprint_.scale.x = 0.01;
    marker_beliefprint_.color.r = 0.69;
    marker_beliefprint_.color.g = 0.88;
    marker_beliefprint_.color.b = 0.9;
    marker_beliefprint_.color.a = 0.5;

    // marker convhull
    marker_convhull_.header.frame_id = frame_id_;
    marker_convhull_.action = visualization_msgs::Marker::ADD;
    marker_convhull_.type = visualization_msgs::Marker::LINE_LIST;
    marker_convhull_.scale.x = 0.06;
    marker_convhull_.color.r = 1.0;
    marker_convhull_.color.g = 0.0;
    marker_convhull_.color.b = 0.0;
    marker_convhull_.color.a = 0.5;
  }

  // visualization
  void polygon_to_marker(const vector<geometry_msgs::Point> &polygon,
                         visualization_msgs::Marker &marker)
  {
    auto point1 = polygon.back();
    for (auto const &point2 : polygon)
    {
      marker.points.push_back(point1);
      marker.points.push_back(point2);
      point1 = point2;
    }
  }

  void visualize(const vector<State_With_Covariance> &path)
  {
    marker_array_swath_.markers.clear();
    marker_array_covariance_.markers.clear();
    marker_array_polyhedron_.markers.clear();
    marker_array_beliefprint_.markers.clear();
    marker_array_convhull_.markers.clear();

    marker_chassis_.points.clear();
    marker_wheels_.points.clear();

    for (int i = 0; i < path.size(); i += 5)
    {
      const auto &state = path[i];
      marker_covariance_.id = i;
      marker_polyhedron_.id = i;
      marker_convhull_.id = i;
      covariance_to_marker(state, marker_covariance_, marker_polyhedron_,
                           marker_array_beliefprint_, marker_convhull_);
      marker_array_covariance_.markers.push_back(marker_covariance_);
      marker_array_polyhedron_.markers.push_back(marker_polyhedron_);
      marker_array_convhull_.markers.push_back(marker_convhull_);

      // visualize true chassis and front wheel steer angle.
      double steer_angle_fl, steer_angle_fr;
      vector<geometry_msgs::Point> wheel_fl, wheel_fr;
      vector<geometry_msgs::Point> oriented_wheel_fl, oriented_wheel_fr;
      vector<geometry_msgs::Point> oriented_footprint;

      // steering angle
      if (fabs(state.state.kappa) > 1e-4)
      {
        steer_angle_fl =
            atan(wheel_fl_pos_.x / ((1 / state.state.kappa) - wheel_fl_pos_.y));
        steer_angle_fr =
            atan(wheel_fr_pos_.x / ((1 / state.state.kappa) - wheel_fr_pos_.y));
      }
      else
      {
        steer_angle_fl = 0.0;
        steer_angle_fr = 0.0;
      }

      // transform wheels and footprint
      costmap_2d::transformFootprint(wheel_fl_pos_.x, wheel_fl_pos_.y,
                                     steer_angle_fl, wheel_, wheel_fl);
      costmap_2d::transformFootprint(wheel_fr_pos_.x, wheel_fr_pos_.y,
                                     steer_angle_fr, wheel_, wheel_fr);
      costmap_2d::transformFootprint(state.state.x, state.state.y,
                                     state.state.theta, wheel_fl,
                                     oriented_wheel_fl);
      costmap_2d::transformFootprint(state.state.x, state.state.y,
                                     state.state.theta, wheel_fr,
                                     oriented_wheel_fr);
      costmap_2d::transformFootprint(state.state.x, state.state.y,
                                     state.state.theta, footprint_,
                                     oriented_footprint);

      polygon_to_marker(oriented_footprint, marker_chassis_);
      polygon_to_marker(oriented_wheel_fl, marker_wheels_);
      polygon_to_marker(oriented_wheel_fr, marker_wheels_);

      // animate
      if (animate_)
      {
        marker_array_swath_.markers.clear();
        marker_array_swath_.markers.push_back(marker_chassis_);
        marker_array_swath_.markers.push_back(marker_wheels_);
        pub_swath_.publish(marker_array_swath_);

        // marker_array_covariance_.markers.clear();
        // marker_array_polyhedron_.markers.clear();
        // marker_array_beliefprint_.markers.clear();
        // marker_array_convhull_.markers.clear();
        pub_covariances_.publish(marker_array_covariance_);
        pub_polyhedrons_.publish(marker_array_polyhedron_);
        pub_beliefprints_.publish(marker_array_beliefprint_);
        pub_convhulls_.publish(marker_array_convhull_);
        ros::spinOnce();
        ros::Duration(0.08).sleep();
      }
    }

    // publish
    if (!animate_)
    {
      marker_array_swath_.markers.push_back(marker_chassis_);
      marker_array_swath_.markers.push_back(marker_wheels_);
      pub_swath_.publish(marker_array_swath_);
      pub_covariances_.publish(marker_array_covariance_);
      pub_polyhedrons_.publish(marker_array_polyhedron_);
      pub_beliefprints_.publish(marker_array_beliefprint_);
      pub_convhulls_.publish(marker_array_convhull_);
      ros::spinOnce();
    }
  }

  // visualization
  void covariance_to_marker(
      const State_With_Covariance &state,
      visualization_msgs::Marker &marker_cov,
      visualization_msgs::Marker &marker_poly,
      visualization_msgs::MarkerArray &marker_belief_array,
      visualization_msgs::Marker &marker_hull)
  {
    marker_cov.pose.position.x = state.state.x;
    marker_cov.pose.position.y = state.state.y;
    marker_cov.pose.position.z = state.state.theta;
    // marker_cov.pose.position.z = 0.0;

    // eigenvalues/-vectors
    Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
    Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());
    Eigen::Matrix3d covariance;
    covariance << state.covariance[0 * 4 + 0], state.covariance[0 * 4 + 1],
        state.covariance[0 * 4 + 2], state.covariance[1 * 4 + 0],
        state.covariance[1 * 4 + 1], state.covariance[1 * 4 + 2],
        state.covariance[2 * 4 + 0], state.covariance[2 * 4 + 1],
        state.covariance[2 * 4 + 2];
    // covariance << 1, 1, 1, 1, 4, 1, 1, 1, 9;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
    if (eigensolver.info() == Eigen::Success)
    {
      eigenvalues = eigensolver.eigenvalues();
      eigenvectors = eigensolver.eigenvectors();
    }
    else
    {
      ROS_WARN(
          "Failed to visualize covariance because eigenvalues can not be "
          "computed.");
    }

    // make right-handed system
    if (eigenvectors.determinant() < 0)
      eigenvectors.col(0) *= -1.0;

    // orientation
    tf::Matrix3x3 rotation(
        eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2),
        eigenvectors(1, 0), eigenvectors(1, 1), eigenvectors(1, 2),
        eigenvectors(2, 0), eigenvectors(2, 1), eigenvectors(2, 2));

    // Eigen::Matrix3d U;
    // U << eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2),
    //     eigenvectors(1, 0), eigenvectors(1, 1), eigenvectors(1, 2),
    //     eigenvectors(2, 0), eigenvectors(2, 1), eigenvectors(2, 2);
    // Eigen::Matrix3d D;
    // D << sqrt(eigenvalues[0]), 0.0, 0.0, 0.0, sqrt(eigenvalues[1]), 0.0, 0.0,
    //     0.0, sqrt(eigenvalues[2]);

    tf::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternionTFToMsg(quaternion, marker_cov.pose.orientation);

    const double confidence = 7.82; // p_CDF: 0.95
    // const double confidence = 11.34; // p_CDF: 0.95
    // const double confidence = 6.25;  // p_CDF: 0.9
    marker_cov.scale.x = 2 * sqrt(confidence * eigenvalues[0]);
    marker_cov.scale.y = 2 * sqrt(confidence * eigenvalues[1]);
    marker_cov.scale.z = 2 * sqrt(confidence * eigenvalues[2]);

    // get icosahedron vertice points
    const double radius = sqrt(confidence);
    const double side_length = 2 * sqrt(3) * radius / pow(tau, 2);
    const auto &vertices = GetMinEnclosingIcosahedronVertices(side_length);
    cout << "side length: " << side_length << ", radius: " << radius << endl;

    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]",
                             "[", "]");
    Eigen::Matrix3d L(covariance.llt().matrixL());
    // cout << "cholesky L: \n" << L.format(HeavyFmt) << endl;
    // // Eigen::Vector3d mu{state.state.x, state.state.y, 0.0};
    Eigen::Vector3d mu{state.state.x, state.state.y, state.state.theta};
    // cout << "covariance: \n" << covariance.format(HeavyFmt) << endl;
    // cout << "cholesky L: \n" << L.format(HeavyFmt) << endl;
    // Eigen::Matrix3d UD = U * D;
    // cout << "SVD U*D: \n" << UD.format(HeavyFmt) << endl;
    // Eigen::Matrix3d LTL = L * L.transpose();
    // Eigen::Matrix3d UDUDT = UD * UD.transpose();
    // cout << "LTL: \n" << LTL.format(HeavyFmt) << endl;
    // cout << "UDUDT: \n" << UDUDT.format(HeavyFmt) << endl;

    const auto &affine_vertices = AffineTransformation(vertices, L, mu);
    VisualizeIcosahedron(affine_vertices, marker_poly);

    VisualizeBeliefPrint(affine_vertices, marker_belief_array, marker_hull);

    // const auto& sample_points = SamplePointsFromEllipsoid(eigenvalues);
    // for (const auto& point : sample_points) {
    //   Eigen::Vector3d rotate_point = eigenvectors * point;
    //   geometry_msgs::Point p;
    //   p.x = rotate_point[0] + state.state.x;
    //   p.y = rotate_point[1] + state.state.y;
    //   p.z = rotate_point[2];
    //   marker_poly.points.push_back(p);
    // }
    // marker_poly.scale.x = 0.01;
    // marker_poly.scale.y = 0.02;
  }

  std::vector<Eigen::Vector3d> AffineTransformation(
      const std::vector<Eigen::Vector3d> &vertices, Eigen::Matrix3d L,
      Eigen::Vector3d mu)
  {
    std::vector<Eigen::Vector3d> affine_vertices;
    for (const auto &vertice : vertices)
    {
      Eigen::Vector3d affine = L * vertice + mu;
      affine_vertices.emplace_back(affine);
    }
    return affine_vertices;
  }

  void VisualizeIcosahedron(const std::vector<Eigen::Vector3d> &vertices,
                            visualization_msgs::Marker &marker)
  {
    if (vertices.size() != 12)
    {
      cout << "Icosahedron vertices number is wrong." << endl;
      return;
    }

    const auto &points = ConvertGeometryPoint(vertices);
    std::vector<geometry_msgs::Point> lines1{
        points[0], points[1], points[0], points[4], points[0],
        points[5], points[0], points[8], points[0], points[9]};
    std::vector<geometry_msgs::Point> lines2{
        points[2], points[3], points[2], points[6], points[2],
        points[7], points[2], points[10], points[2], points[11]};
    std::vector<geometry_msgs::Point> lines3{points[4], points[3], points[4],
                                             points[5], points[4], points[8],
                                             points[4], points[11]};
    std::vector<geometry_msgs::Point> lines4{points[6], points[1], points[6],
                                             points[7], points[6], points[9],
                                             points[6], points[10]};
    std::vector<geometry_msgs::Point> lines5{points[9], points[1], points[9],
                                             points[5], points[9], points[10]};
    std::vector<geometry_msgs::Point> lines6{points[11], points[3], points[11],
                                             points[7], points[11], points[8]};

    std::vector<geometry_msgs::Point> lines7{points[1], points[7], points[1],
                                             points[8], points[7], points[8]};
    std::vector<geometry_msgs::Point> lines8{points[3], points[5], points[3],
                                             points[10], points[5], points[10]};
    marker.points.insert(marker.points.end(), lines1.begin(), lines1.end());
    marker.points.insert(marker.points.end(), lines2.begin(), lines2.end());
    marker.points.insert(marker.points.end(), lines3.begin(), lines3.end());
    marker.points.insert(marker.points.end(), lines4.begin(), lines4.end());
    marker.points.insert(marker.points.end(), lines5.begin(), lines5.end());
    marker.points.insert(marker.points.end(), lines6.begin(), lines6.end());
    marker.points.insert(marker.points.end(), lines7.begin(), lines7.end());
    marker.points.insert(marker.points.end(), lines8.begin(), lines8.end());
  }

  std::vector<geometry_msgs::Point> ConvertGeometryPoint(
      const std::vector<Eigen::Vector3d> &vertices)
  {
    std::vector<geometry_msgs::Point> points;
    for (const auto &vertice : vertices)
    {
      geometry_msgs::Point p;
      p.x = vertice[0];
      p.y = vertice[1];
      p.z = vertice[2];
      points.push_back(p);
    }
    return points;
  }

  std::vector<Eigen::Vector3d> GetMinEnclosingIcosahedronVertices(
      const double a)
  {
    // https://zh.wikipedia.org/wiki/%E6%AD%A3%E4%BA%8C%E5%8D%81%E9%9D%A2%E9%AB%94
    // (0, a/2, a/2*tau) a is side length
    const double b = a / 2;
    const double c = a / 2 * tau;
    std::vector<Eigen::Vector3d> vertices{{0, b, c}, {0, -b, c}, {0, -b, -c}, {0, b, -c}, {b, c, 0}, {-b, c, 0}, {-b, -c, 0}, {b, -c, 0}, {c, 0, b}, {-c, 0, b}, {-c, 0, -b}, {c, 0, -b}};
    return vertices;
  }

  void VisualizeBeliefPrint(
      const std::vector<Eigen::Vector3d> &vertices,
      visualization_msgs::MarkerArray &marker_belief_array,
      visualization_msgs::Marker &marker_hull)
  {
    const auto &points = ConvertGeometryPoint(vertices);
    vector<geometry_msgs::Point> oriented_beliefprint;
    vector<geometry_msgs::Point> all_convhull_points;
    for (const auto &point : points)
    {
      costmap_2d::transformFootprint(point.x, point.y, point.z, footprint_,
                                     oriented_beliefprint);
      marker_beliefprint_.points.clear();
      polygon_to_marker(oriented_beliefprint, marker_beliefprint_);
      all_convhull_points.insert(all_convhull_points.end(),
                                 oriented_beliefprint.begin(),
                                 oriented_beliefprint.end());

      // visualize
      if (belief_cnt % 12 == 0)
      {
        marker_beliefprint_.scale.x = 0.01;
        marker_beliefprint_.color.r = random(0.0, 1.0);
        marker_beliefprint_.color.g = random(0.0, 1.0);
        marker_beliefprint_.color.b = random(0.0, 1.0);
      }
      marker_beliefprint_.id = belief_cnt;
      marker_belief_array.markers.push_back(marker_beliefprint_);
      belief_cnt++;
    }

    cout << "all_convhull_points size: " << all_convhull_points.size() << endl;
    marker_hull.points.clear();
    ComputeConvexHull(all_convhull_points, marker_hull);
  }

  double CrossProd(const geometry_msgs::Point &start_point,
                   const geometry_msgs::Point &end_point_1,
                   const geometry_msgs::Point &end_point_2)
  {
    double a_x = end_point_1.x - start_point.x;
    double a_y = end_point_1.y - start_point.y;
    double b_x = end_point_2.x - start_point.x;
    double b_y = end_point_2.y - start_point.y;
    // geometry_msgs::Point a = end_point_1 - start_point;
    // geometry_msgs::Point b = end_point_2 - start_point;
    return a_x * b_y - a_y * b_x;
    // return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
  }

  bool ComputeConvexHull(const std::vector<geometry_msgs::Point> &points,
                         visualization_msgs::Marker &marker)
  {
    const int n = static_cast<int>(points.size());
    if (n < 3)
    {
      return false;
    }
    std::vector<int> sorted_indices(n);
    for (int i = 0; i < n; ++i)
    {
      sorted_indices[i] = i;
    }
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](const int idx1, const int idx2)
              {
                const geometry_msgs::Point &pt1 = points[idx1];
                const geometry_msgs::Point &pt2 = points[idx2];
                const double dx = pt1.x - pt2.x;
                if (std::abs(dx) > kMathEpsilon)
                {
                  return dx < 0.0;
                }
                return pt1.y < pt2.y;
              });
    int count = 0;
    std::vector<int> results;
    results.reserve(n);
    int last_count = 1;
    for (int i = 0; i < n + n; ++i)
    {
      if (i == n)
      {
        last_count = count;
      }
      const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
      const geometry_msgs::Point &pt = points[idx];
      while (count > last_count &&
             CrossProd(points[results[count - 2]], points[results[count - 1]],
                       pt) <= kMathEpsilon)
      {
        results.pop_back();
        --count;
      }
      results.push_back(idx);
      ++count;
    }
    --count;
    if (count < 3)
    {
      return false;
    }
    std::vector<geometry_msgs::Point> result_points;
    result_points.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      result_points.push_back(points[results[i]]);
    }

    cout << "result_points size: " << result_points.size() << endl;
    auto p1 = result_points.back();
    for (auto const &p2 : result_points)
    {
      marker.points.push_back(p1);
      marker.points.push_back(p2);
      p1 = p2;
    }
    return true;
  }

  std::vector<Eigen::Vector3d> SamplePointsFromEllipsoid(
      const Eigen::Vector3d &eigenvalues)
  {
    const int N = 16;
    const double a = 3 * sqrt(eigenvalues[0]);
    const double b = 3 * sqrt(eigenvalues[1]);
    const double c = 3 * sqrt(eigenvalues[2]);

    // use min Icosahedron to enclose this ellipse
    // int cnt = 0;
    std::vector<Eigen::Vector3d> sample_points;
    sample_points.emplace_back(0, 0, c);
    for (double phi = -M_PI; phi < M_PI; phi += M_PI / 3.0)
    {
      for (double theta = M_PI / 4.0; theta < M_PI; theta += M_PI / 4.0)
      {
        double x = a * sin(theta) * cos(phi);
        double y = b * sin(theta) * sin(phi);
        double z = c * cos(theta);
        sample_points.emplace_back(x, y, z);
        // cnt++;
      }
    }

    sample_points.emplace_back(0, 0, -c);
    // cout << "sample points size: " << sample_points.size() << endl;
    return sample_points;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steering_functions");

  RobotClass robot;

  // sample
  int seed(ros::Time::now().toSec());
  srand(seed);
  while (ros::ok())
  {
    State_With_Covariance start;
    // start.state.x = random(-OPERATING_REGION_X / 2.0, OPERATING_REGION_X
    // / 2.0); start.state.y = random(-OPERATING_REGION_Y / 2.0,
    // OPERATING_REGION_Y / 2.0); start.state.theta =
    // random(-OPERATING_REGION_THETA / 2.0, OPERATING_REGION_THETA / 2.0);
    // start.state.kappa = random(-robot.kappa_max_, robot.kappa_max_);
    start.state.x = 0.0;
    start.state.y = 0.0;
    start.state.theta = 0.0;
    start.state.d = 0.0;
    // start.covariance[0 * 4 + 0] = start.Sigma[0 * 4 + 0] =
    //     pow(robot.measurement_noise_.std_x, 2);
    // start.covariance[1 * 4 + 1] = start.Sigma[1 * 4 + 1] =
    //     pow(robot.measurement_noise_.std_y, 2);
    // start.covariance[2 * 4 + 2] = start.Sigma[2 * 4 + 2] =
    //     pow(robot.measurement_noise_.std_theta, 2);
    start.covariance[0 * 4 + 0] = start.Sigma[0 * 4 + 0] = pow(0.0, 2);
    start.covariance[1 * 4 + 1] = start.Sigma[1 * 4 + 1] = pow(0.0, 2);
    start.covariance[2 * 4 + 2] = start.Sigma[2 * 4 + 2] = pow(0.0, 2);

    // State_With_Covariance start_wout_curv;
    // start_wout_curv.state.x = start.state.x;
    // start_wout_curv.state.y = start.state.y;
    // start_wout_curv.state.theta = start.state.theta;
    // start_wout_curv.state.kappa = 0.0;
    // start_wout_curv.state.d = start.state.d;
    // copy(&start.covariance[0], &start.covariance[15],
    // &start_wout_curv.covariance[0]); copy(&start.Sigma[0], &start.Sigma[15],
    // &start_wout_curv.Sigma[0]);

    State goal;
    // goal.x = random(-OPERATING_REGION_X / 2.0, OPERATING_REGION_X / 2.0);
    // goal.y = random(-OPERATING_REGION_Y / 2.0, OPERATING_REGION_Y / 2.0);
    // goal.theta = random(-OPERATING_REGION_THETA / 2.0, OPERATING_REGION_THETA
    // / 2.0); goal.kappa = random(-robot.kappa_max_, robot.kappa_max_);
    goal.x = 0.0;
    goal.y = 11.0;
    goal.theta = M_PI;
    goal.kappa = 0.0;
    goal.d = 0.0;

    // State goal_wout_curv;
    // goal_wout_curv.x = goal.x;
    // goal_wout_curv.y = goal.y;
    // goal_wout_curv.theta = goal.theta;
    // goal_wout_curv.kappa = 0.0;
    // goal_wout_curv.d = goal.d;

    // PathClass cc_dubins_path("CC_Dubins", start, goal, robot.kappa_max_,
    //                          robot.sigma_max_);

    // PathClass cc00_dubins_path("CC00_Dubins", start_wout_curv,
    // goal_wout_curv, robot.kappa_max_, robot.sigma_max_); PathClass
    // cc0pm_dubins_path("CC0pm_Dubins", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass
    // ccpm0_dubins_path("CCpm0_Dubins", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass
    // ccpmpm_dubins_path("CCpmpm_Dubins", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass dubins_path("Dubins",
    // start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    // PathClass cc00_rs_path("CC00_RS", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_);

    PathClass hc_rs_path("HC_RS", start, goal, robot.kappa_max_,
                         robot.sigma_max_);

    // PathClass hc00_rs_path("HC00_RS", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass hc0pm_rs_path("HC0pm_RS",
    // start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    // PathClass hcpm0_rs_path("HCpm0_RS", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass
    // hcpmpm_rs_path("HCpmpm_RS", start_wout_curv, goal_wout_curv,
    // robot.kappa_max_, robot.sigma_max_); PathClass rs_path("RS",
    // start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);

    // visualize
    // cc_dubins_path.visualize();
    // robot.visualize(cc_dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // cc00_dubins_path.visualize();
    // robot.visualize(cc00_dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // cc0pm_dubins_path.visualize();
    // robot.visualize(cc0pm_dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // ccpm0_dubins_path.visualize();
    // robot.visualize(ccpm0_dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // ccpmpm_dubins_path.visualize();
    // robot.visualize(ccpmpm_dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // dubins_path.visualize();
    // robot.visualize(dubins_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // cc00_rs_path.visualize();
    // robot.visualize(cc00_rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    hc_rs_path.visualize();
    robot.visualize(hc_rs_path.path_);
    ros::Duration(VISUALIZATION_DURATION).sleep();

    // hc00_rs_path.visualize();
    // robot.visualize(hc00_rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // hc0pm_rs_path.visualize();
    // robot.visualize(hc0pm_rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // hcpm0_rs_path.visualize();
    // robot.visualize(hcpm0_rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // hcpmpm_rs_path.visualize();
    // robot.visualize(hcpmpm_rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();

    // rs_path.visualize();
    // robot.visualize(rs_path.path_);
    // ros::Duration(VISUALIZATION_DURATION).sleep();
  }
  return 0;
}
