/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Recovery behavior of step back and steer turning
 *
 * \author Masaru Morita
 */

#ifndef STEP_BACK_AND_MAX_STEER_RECOVERY_H
#define STEP_BACK_AND_MAX_STEER_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;
namespace blp=base_local_planner;
using std::vector;
using std::max;

namespace step_back_and_max_steer_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class StepBackAndMaxSteerRecovery : public nav_core::RecoveryBehavior
{
public:
  
  /// Doesn't do anything: initialize is where the actual work happens
  StepBackAndMaxSteerRecovery();

  ~StepBackAndMaxSteerRecovery();

  /// Initialize the parameters of the behavior
  void initialize (std::string n, tf::TransformListener* tf,
                   costmap_2d::Costmap2DROS* global_costmap,
                   costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  void runBehavior();

private:
  enum COSTMAP_SEARCH_MODE
  {
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD
  };

  enum TURN_DIRECTION
  {
    LEFT,
    RIGHT,
  };

  gm::Twist TWIST_STOP;

  gm::Pose2D getCurrentLocalPose () const;
  gm::Twist scaleGivenAccelerationLimits (const gm::Twist& twist, const double time_remaining) const;
  double nonincreasingCostInterval (const gm::Pose2D& current, const gm::Twist& twist) const;
  double normalizedPoseCost (const gm::Pose2D& pose) const;
  gm::Twist transformTwist (const gm::Pose2D& pose) const;
  void moveSpacifiedLength (const gm::Twist twist, const double duaration) const;
  void moveSpacifiedLength (const gm::Twist twist, double length, COSTMAP_SEARCH_MODE mode = FORWARD);
  double getCurrentDiff(const gm::Pose2D initialPose, COSTMAP_SEARCH_MODE mode = FORWARD);
  double getCurrentDistDiff(const gm::Pose2D initialPose, const double distination, COSTMAP_SEARCH_MODE mode = FORWARD);
  double getMinimalDistance(const COSTMAP_SEARCH_MODE mode);
  int determineTurnDirection();


  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* global_costmap_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2D costmap_; // Copy of local_costmap_, used by world model
  std::string name_;
  tf::TransformListener* tf_;
  ros::Publisher pub_;
  bool initialized_;

  // Memory owned by this object
  // Mutable because footprintCost is not declared const
  mutable base_local_planner::CostmapModel* world_model_;

  gm::Twist base_frame_twist_;
  
  double duration_;
  double linear_speed_limit_;
  double angular_speed_limit_;
  double linear_acceleration_limit_;
  double angular_acceleration_limit_;
  double controller_frequency_;
  double simulation_inc_;

  bool only_single_steering_;
  int trial_times_;
  double obstacle_patience_;
  //-- back
  double linear_vel_back_;
  double step_back_length_;
  //-- steer
  double linear_vel_steer_;
  double angular_speed_steer_;
  double turn_angle_;
  //-- forward
  double linear_vel_forward_;
  double step_forward_length_;

};

} // namespace step_back_and_max_steer_recovery

#endif // include guard
