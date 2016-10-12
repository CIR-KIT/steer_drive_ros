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
 * \author Bhaskara Marthi
 * 
 */

#include <step_back_and_max_steer_recovery/step_back_and_max_steer_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(step_back_and_max_steer_recovery, StepBackAndMaxSteerRecovery, step_back_and_max_steer_recovery::StepBackAndMaxSteerRecovery,      
                        nav_core::RecoveryBehavior)

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;
namespace blp=base_local_planner;
using std::vector;
using std::max;

namespace step_back_and_max_steer_recovery
{

StepBackAndMaxSteerRecovery::StepBackAndMaxSteerRecovery () :
  global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)
{}

StepBackAndMaxSteerRecovery::~StepBackAndMaxSteerRecovery ()
{
  delete world_model_;
}

void StepBackAndMaxSteerRecovery::initialize (std::string name, tf::TransformListener* tf,
                                cmap::Costmap2DROS* global_cmap, cmap::Costmap2DROS* local_cmap)
{
  ROS_ASSERT(!initialized_);
  name_ = name;
  tf_ = tf;
  local_costmap_ = local_cmap;
  global_costmap_ = global_cmap;
  /*
  local_costmap_->getCostmapCopy(costmap_);
  world_model_ = new blp::CostmapModel(costmap_);
  */
  world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

  pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
  ros::NodeHandle private_nh("~/" + name);

  {
  bool found=true;
  found = found && private_nh.getParam("linear_x", base_frame_twist_.linear.x);
  found = found && private_nh.getParam("linear_y", base_frame_twist_.linear.y);
  found = found && private_nh.getParam("angular_z", base_frame_twist_.angular.z);
  if (!found) {
    ROS_FATAL_STREAM ("Didn't find twist parameters in " << private_nh.getNamespace());
    ros::shutdown();
  }
  }

  private_nh.param("duration", duration_, 1.0);
  private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
  private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
  private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
  private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("simulation_inc", simulation_inc_, 1/controller_frequency_);

  duration_ = 3.0;

  ROS_INFO_STREAM_NAMED ("top", "Initialized twist recovery with twist " <<
                          base_frame_twist_ << " and duration " << duration_);
  
  initialized_ = true;
}

gm::Twist scaleTwist (const gm::Twist& twist, const double scale)
{
  gm::Twist t;
  t.linear.x = twist.linear.x * scale;
  t.linear.y = twist.linear.y * scale;
  t.angular.z = twist.angular.z * scale;
  return t;
}

gm::Pose2D forwardSimulate (const gm::Pose2D& p, const gm::Twist& twist, const double t=1.0)
{
  gm::Pose2D p2;
  const double linear_vel = twist.linear.x;//sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
  p2.x = p.x + linear_vel * cos(twist.angular.z)*t;
  p2.y = p.y + linear_vel * sin(twist.angular.z)*t;
  //p2.y = p.y + twist.linear.y*t;
  p2.theta = p.theta + twist.angular.z*t;
  return p2;
}

/// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
double StepBackAndMaxSteerRecovery::normalizedPoseCost (const gm::Pose2D& pose) const
{
  gm::Point p;
  p.x = pose.x;
  p.y = pose.y;
  vector<gm::Point> oriented_footprint;// = local_costmap_->getRobotFootprint();
  //local_costmap_->getOrientedFootprint(pose.x, pose.y, pose.theta, oriented_footprint);
  local_costmap_->getOrientedFootprint(oriented_footprint);
  const double c = world_model_->footprintCost(p, oriented_footprint, 5.0, 5.0);
  //double cost = local_costmap_->getCostmap()->getCost(pose.x, pose.y);
  //return cost < 0 ? 1e9 : cost;
  return c < 0 ? 1e9 : c;
}


/// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
/// d seconds if we follow twist
/// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
/// the first k of those d seconds, but this is not done
double StepBackAndMaxSteerRecovery::nonincreasingCostInterval (const gm::Pose2D& current, const gm::Twist& twist) const
{
  double cost = normalizedPoseCost(current);
  double t; // Will hold the first time that is invalid
  gm::Pose2D current_tmp = current;
  for (t=simulation_inc_; t<=duration_; t+=simulation_inc_) {
    current_tmp = forwardSimulate(current_tmp, twist, simulation_inc_);
    const double next_cost = normalizedPoseCost(current_tmp);
    if (next_cost > cost) {
      ROS_INFO_NAMED ("top", " ");
      ROS_INFO_NAMED ("top", "cost = %.2f, next_cost = %.2f", cost, next_cost);
      ROS_INFO_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist.linear.x, twist.angular.z);
      ROS_INFO_NAMED ("top", "init = (%.2f, %.2f), current = (%.2f, %.2f)", current.x, current.y, current_tmp.x, current_tmp.y);
      ROS_INFO_NAMED ("top", "time = %.2f", t);
      ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t << " and pose " << forwardSimulate(current, twist, t)
                              << " is " << next_cost << " which is greater than previous cost " << cost);
      break;
    }
    cost = next_cost;
  }
  ROS_INFO_NAMED ("top", " ");
  ROS_INFO_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist.linear.x, twist.angular.z);
  ROS_INFO_NAMED ("top", "init = (%.2f, %.2f), current = (%.2f, %.2f)", current.x, current.y, current_tmp.x, current_tmp.y);

  for (double t2=simulation_inc_; t2<=100.0; t2+=simulation_inc_) {
    gm::Twist twist_foward;
    twist_foward.linear.x = twist.linear.x;
    current_tmp = forwardSimulate(current_tmp, twist_foward, simulation_inc_);
    const double next_cost = normalizedPoseCost(current_tmp);
    if (next_cost > cost) {
      ROS_INFO_NAMED ("top", "fowardsim");
      ROS_INFO_NAMED ("top", "cost = %.2f, next_cost = %.2f", cost, next_cost);
      ROS_INFO_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist_foward.linear.x, twist_foward.angular.z);
      ROS_INFO_NAMED ("top", "init = (%.2f, %.2f), current = (%.2f, %.2f)", current.x, current.y, current_tmp.x, current_tmp.y);
      ROS_INFO_NAMED ("top", "time = %.2f", t2);
      ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t2 << " and pose " << forwardSimulate(current, twist_foward, t2)
                              << " is " << next_cost << " which is greater than previous cost " << cost);
      break;
    }
    cost = next_cost;
  }
  return t-simulation_inc_;
}

double linearSpeed (const gm::Twist& twist)
{
  return sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
}

double angularSpeed (const gm::Twist& twist)
{
  return fabs(twist.angular.z);
}

// Scale twist so we can stop in the given time, and so it's within the max velocity
gm::Twist StepBackAndMaxSteerRecovery::scaleGivenAccelerationLimits (const gm::Twist& twist, const double time_remaining) const
{
  const double linear_speed = linearSpeed(twist);
  const double angular_speed = angularSpeed(twist);
  const double linear_acc_scaling = linear_speed/(time_remaining*linear_acceleration_limit_);
  const double angular_acc_scaling = angular_speed/(time_remaining*angular_acceleration_limit_);
  const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
  const double linear_vel_scaling = linear_speed/linear_speed_limit_;
  const double angular_vel_scaling = angular_speed/angular_speed_limit_;
  const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
  return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
}

// Get pose in local costmap frame
gm::Pose2D StepBackAndMaxSteerRecovery::getCurrentLocalPose () const
{
  tf::Stamped<tf::Pose> p;
  local_costmap_->getRobotPose(p);
  gm::Pose2D pose;
  pose.x = p.getOrigin().x();
  pose.y = p.getOrigin().y();
  pose.theta = tf::getYaw(p.getRotation());
  return pose;
}

void StepBackAndMaxSteerRecovery::runBehavior ()
{
  ROS_ASSERT (initialized_);

  // Figure out how long we can safely run the behavior
  const gm::Pose2D& current = getCurrentLocalPose();
  //local_costmap_->getCostmapCopy(costmap_); // This affects world_model_, which is used in the next step
  // this should be affected automatically

  base_frame_twist_.angular.z = 0.3;
  const double d = nonincreasingCostInterval(current, base_frame_twist_);
  ros::Rate r(controller_frequency_);
  ROS_INFO_NAMED ("top", "Applying (%.2f, %.2f, %.2f) for %.2f seconds", base_frame_twist_.linear.x,
                   base_frame_twist_.linear.y, base_frame_twist_.angular.z, d);

  for (double t=0; t<d; t+=1/controller_frequency_) {
    pub_.publish(scaleGivenAccelerationLimits(base_frame_twist_, d-t));
    r.sleep();
  }

  gm::Twist twist_stop;
  pub_.publish(twist_stop);
  ROS_INFO_NAMED ("top", "stop");
  for(int i = 0; i < 50; i++)
    r.sleep();

  const gm::Pose2D& current2 = getCurrentLocalPose();

  gm::Twist twist_f, twist_l, twist_r;
  twist_f.linear.x = -0.3;
  const double d_f = nonincreasingCostInterval(current2, twist_f);
  ROS_INFO_NAMED ("top", "forward (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_f.linear.x, twist_f.linear.y, twist_f.angular.z, d_f);
  double d_l;
  twist_l.linear.x = 0.3;
  twist_l.angular.z = 0.05;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_l.angular.z = 0.1;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_l.angular.z = 0.15;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_l.angular.z = 0.2;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  /*
  twist_l.angular.z = 0.3;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_l.angular.z = 0.4;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_l.angular.z = 0.5;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
                  */

  double d_r;
  twist_r.linear.x = 0.3;
  twist_r.angular.z = -0.05;
  d_r = nonincreasingCostInterval(current2, twist_r);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_r.linear.x, twist_r.linear.y, twist_r.angular.z, d_r);
  twist_r.angular.z = -0.1;
  d_r = nonincreasingCostInterval(current2, twist_r);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_r.linear.x, twist_r.linear.y, twist_r.angular.z, d_r);
  twist_r.angular.z = -0.15;
  d_r = nonincreasingCostInterval(current2, twist_r);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_r.linear.x, twist_r.linear.y, twist_r.angular.z, d_r);
  twist_l.angular.z = -0.2;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  /*
  twist_r.angular.z = -0.3;
  d_r = nonincreasingCostInterval(current2, twist_r);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_r.linear.x, twist_r.linear.y, twist_r.angular.z, d_r);
  twist_l.angular.z = -0.4;
  d_l = nonincreasingCostInterval(current2, twist_l);
  ROS_INFO_NAMED ("top", "left (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_l.linear.x, twist_l.linear.y, twist_l.angular.z, d_l);
  twist_r.angular.z = -0.5;
  d_r = nonincreasingCostInterval(current2, twist_r);
  ROS_INFO_NAMED ("top", "right (%.2f, %.2f, %.2f) for %.2f seconds",
                  twist_r.linear.x, twist_r.linear.y, twist_r.angular.z, d_r);
                  */
  gm::Twist twist;
  twist.linear.x = 0.3;
  /*
  twist.angular.z = 0.2;

  for (double t=0; t<d; t+=1/controller_frequency_) {
    pub_.publish(scaleGivenAccelerationLimits(twist, d-t));
    r.sleep();
  }

  pub_.publish(twist_stop);
  ROS_INFO_NAMED ("top", "stop");
  for(int i = 0; i < 50; i++)
    r.sleep();
    */

  twist.angular.z = 0.0;
  for (double t=0; t<4; t+=1/controller_frequency_) {
    pub_.publish(scaleGivenAccelerationLimits(twist, d-t));
    r.sleep();
  }

  pub_.publish(twist_stop);
  ROS_INFO_NAMED ("top", "stop");
  for(int i = 0; i < 50; i++)
    r.sleep();

  /*
  // We'll now apply this twist open-loop for d seconds (scaled so we can guarantee stopping at the end)
  for (double t=0; t<d; t+=1/controller_frequency_) {
    pub_.publish(scaleGivenAccelerationLimits(base_frame_twist_, d-t));
    r.sleep();
  }

  pub_.publish(twist_stop);
  ROS_INFO_NAMED ("top", "stop");
  for(int i = 0; i < 50; i++)
    r.sleep();
    */

}


} // namespace step_back_and_max_steer_recovery
