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

namespace step_back_and_max_steer_recovery
{

StepBackAndMaxSteerRecovery::StepBackAndMaxSteerRecovery () :
  global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)
{
    TWIST_STOP.linear.x = 0.0;
    TWIST_STOP.linear.y = 0.0;
    TWIST_STOP.linear.z = 0.0;
    TWIST_STOP.angular.x = 0.0;
    TWIST_STOP.angular.y = 0.0;
    TWIST_STOP.angular.z = 0.0;
}

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

  /*
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
  */

  private_nh.param("duration", duration_, 1.0);
  private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
  private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
  private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
  private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("simulation_inc", simulation_inc_, 1/controller_frequency_);

  private_nh.param("global_frame", global_frame_, std::string("/odom"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("/base_link"));
  private_nh.param("only_single_steering", only_single_steering_, true);
  // back
  private_nh.param("linear_vel_back", linear_vel_back_, -0.3);
  private_nh.param("duration_back", duration_back_, 3.0);
  //-- steer
  private_nh.param("linear_vel_steer", linear_vel_steer_, 0.3);
  private_nh.param("angular_vel_steer", angular_vel_steer_, 0.5);
  private_nh.param("duration_steer", duration_steer_, 2.0);
  //-- forward
  private_nh.param("linear_vel_forward", linear_vel_forward_, 0.3);
  private_nh.param("duration_forward", duration_forward_, 3.0);

  /*
  ROS_INFO_STREAM_NAMED ("top", "Initialized twist recovery with twist " <<
                          base_frame_twist_ << " and duration " << duration_);
  */

  ROS_INFO_NAMED ("top", "Initialized with global_frame = %s, robot_base_frame = %s", global_frame_.c_str(), robot_base_frame_.c_str());
  ROS_INFO_NAMED ("top", "Initialized with trial_times = %d", trial_times_);
  ROS_INFO_NAMED ("top", "Initialized with only_single_steering = %s", (only_single_steering_ ? "true" : "false"));
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_back = %.2f, duration_back = %.2f",
                  linear_vel_back_, duration_back_);
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_steer = %.2f, angular_vel_steer = %.2f, duration_steer = %.2f",
                  linear_vel_steer_, angular_vel_steer_, duration_steer_);
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_forward = %.2f, duration_forward = %.2f",
                  linear_vel_forward_, duration_forward_);

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
  const double linear_vel = twist.linear.x;
  p2.theta = p.theta + twist.angular.z;//*t;
  p2.x = p.x + linear_vel * cos(p2.theta)*t;
  p2.y = p.y + linear_vel * sin(p2.theta)*t;

  return p2;
}

/// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
double StepBackAndMaxSteerRecovery::normalizedPoseCost (const gm::Pose2D& pose) const
{
  gm::Point p;
  p.x = pose.x;
  p.y = pose.y;

  unsigned int pose_map_idx_x, pose_map_idx_y;
  costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
  costmap->worldToMap(p.x, p.y, pose_map_idx_x, pose_map_idx_y);  // convert point unit from [m] to [idx]
  ROS_DEBUG_NAMED ("top", "Trying to get cost at (%d, %d) in getCost", pose_map_idx_x, pose_map_idx_y);
  const double c = costmap->getCost(pose_map_idx_x, pose_map_idx_y);

  return c < 0 ? 1e9 : c;
}


/// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
/// d seconds if we follow twist
/// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
/// the first k of those d seconds, but this is not done
double StepBackAndMaxSteerRecovery::nonincreasingCostInterval (const gm::Pose2D& current, const gm::Twist& twist) const
{
  double cost = 0;
  cost = normalizedPoseCost(current);
  double t; // Will hold the first time that is invalid
  gm::Pose2D current_tmp = current;
  double next_cost;

  ROS_DEBUG_NAMED ("top", " ");
  for (t=simulation_inc_; t<=duration_ + 1000; t+=simulation_inc_) {
      ROS_DEBUG_NAMED ("top", "start loop");
      current_tmp = forwardSimulate(current, twist, t);
      ROS_DEBUG_NAMED ("top", "finish fowardSimulate");
      next_cost = normalizedPoseCost(current_tmp);
      ROS_DEBUG_NAMED ("top", "finish Cost");
    if (next_cost > cost) {
    //if (/*next_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||*/ next_cost == costmap_2d::LETHAL_OBSTACLE) {
      ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t << " and pose " << forwardSimulate(current, twist, t)
                              << " is " << next_cost << " which is greater than previous cost " << cost);
      break;
    }
    cost = next_cost;
  }
  ROS_DEBUG_NAMED ("top", "cost = %.2f, next_cost = %.2f", cost, next_cost);
  ROS_DEBUG_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist.linear.x, twist.angular.z);
  ROS_DEBUG_NAMED ("top", "init = (%.2f, %.2f, %.2f), current = (%.2f, %.2f, %.2f)",
                  current.x, current.y, current.theta, current_tmp.x, current_tmp.y, current_tmp.theta);
  ROS_DEBUG_NAMED ("top", "time = %.2f", t);

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

void StepBackAndMaxSteerRecovery::moveSpacifiedLength (const gm::Twist twist, const double duaration) const
{
    ros::Rate r(controller_frequency_);

    gm::Twist twist_stop;
    twist_stop.linear.x = 0.0;
    twist_stop.linear.y = 0.0;
    twist_stop.linear.z = 0.0;
    twist_stop.angular.x = 0.0;
    twist_stop.angular.y = 0.0;
    twist_stop.angular.z = 0.0;

    for (double t=0; t<duaration; t+=1/controller_frequency_) {
        // TODO: obstacle detect and stop required
        const gm::Pose2D& current_tmp = getCurrentLocalPose();
        double time_until_obstacle = nonincreasingCostInterval(current_tmp, twist);
        if(time_until_obstacle < 1.0)
            pub_.publish(scaleGivenAccelerationLimits(twist_stop, duaration-t));
        else
            pub_.publish(scaleGivenAccelerationLimits(twist, duaration-t));

        r.sleep();
    }

    return;
}

void StepBackAndMaxSteerRecovery::moveSpacifiedLength (const gm::Twist twist, const gm::Pose2D initialPose, double distination, COSTMAP_SEARCH_MODE mode)
{
    ros::Rate r(controller_frequency_);

    while (double dist_diff = getCurrentDistDiff(initialPose, distination) > 0.01)
    {
        double remaining_time = dist_diff / base_frame_twist_.linear.x;

        //if(dist_to_obstacle < 0.5)
        if(double min_dist = getMinimalDistance(mode) < 1.1)
        {
            pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, remaining_time));
            ROS_INFO_NAMED ("top", "obstacle detected");
            ROS_WARN_NAMED ("top", "getMinimalDistance = %f", min_dist);
            break;
        }

        pub_.publish(scaleGivenAccelerationLimits(twist, remaining_time));
        ROS_INFO_NAMED ("top", "no obstacle");
        ROS_INFO_NAMED ("top", "getMinimalDistance = %f", getMinimalDistance(mode));

        r.sleep();
    }

    return;
}


double StepBackAndMaxSteerRecovery::getTranslation(double x0, double y0) const
{
    /*
    tf::StampedTransform transform;
    tf_->lookupTransform(global_frame_, robot_base_frame_,ros::Time(0), transform);
    geometry_msgs::TransformStamped msg;
    tf::transformStampedTFToMsg(transform, msg);
    double translation = (msg.transform.translation.x -x0)*(msg.transform.translation.x -x0) + (msg.transform.translation.y -y0)*(msg.transform.translation.y -y0);
    */
    const gm::Pose2D currentPos = getCurrentLocalPose();
    double translation = (currentPos.x -x0)*(currentPos.x -x0) + (currentPos.y -y0)*(currentPos.y -y0);
    translation = sqrt(translation);
    return translation;
}

gm::Pose2D StepBackAndMaxSteerRecovery::getCurrentTFPos() const
{
    tf::StampedTransform transform;
    tf_->lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
    geometry_msgs::TransformStamped msg;
    tf::transformStampedTFToMsg(transform, msg);

    gm::Pose2D currentPos;
    currentPos.x = msg.transform.translation.x;
    currentPos.y = msg.transform.translation.y;

    return currentPos;
}

double StepBackAndMaxSteerRecovery::getCurrentDiff(const gm::Pose2D initialPose)
{

    const gm::Pose2D& currentPose = getCurrentLocalPose();
    ROS_DEBUG_NAMED ("top", "current pose (%.2f, %.2f, %.2f)", currentPose.x,
                       currentPose.y, currentPose.theta);

    double current_diff = (currentPose.x - initialPose.x)*(currentPose.x - initialPose.x) +
                       (currentPose.y - initialPose.y)*(currentPose.y - initialPose.y);

    current_diff = sqrt(current_diff);
    ROS_DEBUG_NAMED ("top", "current_diff = %.2f", current_diff);

    return current_diff;
}

double StepBackAndMaxSteerRecovery::getCurrentDistDiff(const gm::Pose2D initialPose, const double dist_length)
{
    const double dist_diff = dist_length - getCurrentDiff(initialPose);
    ROS_DEBUG_NAMED ("top", "dist_diff = %.2f", dist_diff);

    return dist_diff;
}

double StepBackAndMaxSteerRecovery::getMinimalDistance(const COSTMAP_SEARCH_MODE mode)
{
    double max_angle, min_angle;
    gm::Twist twist;

    switch (mode) {
    case FOWARD:
        twist.linear.x = linear_vel_forward_;
        max_angle = M_PI/2.0;
        min_angle = -M_PI/2.0;
        break;
    case FOWARD_LEFT:
        twist.linear.x = linear_vel_forward_;
        max_angle = M_PI/2.0;
        min_angle = 0.0;
        break;
    case FOWARD_RIGHT:
        max_angle = 0.0;
        min_angle = -M_PI/2.0;
        break;
    case BACKWARD:
        twist.linear.x = linear_vel_back_;
        max_angle = M_PI/3.0;
        min_angle = -M_PI/3.0;
        break;
    default:
        break;
    }

    const gm::Pose2D& current = getCurrentLocalPose();
    double min_time = INFINITY;

    for(double angle = min_angle; angle < max_angle; angle+=0.1)
      {
          twist.angular.z = angle;
          double time_until_obstacle = nonincreasingCostInterval(current, twist);
          ROS_DEBUG_NAMED ("top", "(%.2f, %.2f, %.2f) for %.2f seconds",
                          twist.linear.x, twist.linear.y, twist.angular.z, time_until_obstacle);

          if(time_until_obstacle < min_time)
              min_time = time_until_obstacle;
    }

    double min_dist = abs(twist.linear.x * min_time);

    return min_dist;
}

void StepBackAndMaxSteerRecovery::runBehavior ()
{
  ROS_ASSERT (initialized_);

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "**********Start StepBackAndSteerRecovery!!!**********");
  ROS_INFO_NAMED ("top", "*****************************************************");

  int cnt = 0;
  while(true)
  {
      cnt++;
      // Figure out how long we can safely run the behavior
      const gm::Pose2D& initialPose = getCurrentLocalPose();
      //local_costmap_->getCostmapCopy(costmap_); // This affects world_model_, which is used in the next step
      // this should be affected automatically

      // step back
      base_frame_twist_.linear.x = linear_vel_back_;
      base_frame_twist_.angular.z = 0.0;
      double d = nonincreasingCostInterval(initialPose, base_frame_twist_);
      ROS_DEBUG_NAMED ("top", "initial pose (%.2f, %.2f, %.2f)", initialPose.x,
                       initialPose.y, initialPose.theta);
      ros::Rate r(controller_frequency_);
      ROS_DEBUG_NAMED ("top", "Applying (%.2f, %.2f, %.2f) for %.2f seconds", base_frame_twist_.linear.x,
                       base_frame_twist_.linear.y, base_frame_twist_.angular.z, d);


      // step back
      double step_back_length = 2.0;

      moveSpacifiedLength(base_frame_twist_, initialPose, step_back_length, BACKWARD);

      double final_diff = getCurrentDiff(initialPose);
      ROS_DEBUG_NAMED ("top", "final_diff = %.2f",final_diff);

      // stop
      d = 1.0;
      for (double t=0; t<d; t+=1/controller_frequency_) {
          pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, d-t));
          r.sleep();
      }

      // simulate and evaluate cost
      const gm::Pose2D& current2 = getCurrentLocalPose();

      gm::Twist twist;
      twist.linear.x = linear_vel_forward_;
      //twist.linear.x = 0.3;

      vector<double> times_until_obstacle_r;
      vector<double> times_until_obstacle_l;
      double max = 1.0;
      double min = - max;
      for(double angle = min; angle < max; angle+=0.1)
      {
          //if (angle == 0.0)
          //    continue;

          twist.angular.z = angle;
          double time_until_obstacle = nonincreasingCostInterval(current2, twist);
          ROS_DEBUG_NAMED ("top", "(%.2f, %.2f, %.2f) for %.2f seconds",
                          twist.linear.x, twist.linear.y, twist.angular.z, time_until_obstacle);

          if (time_until_obstacle > 50)
              time_until_obstacle = 50;

          if(angle > 0.0)
              times_until_obstacle_l.push_back(time_until_obstacle);
          else if(angle < 0.0)
              times_until_obstacle_r.push_back(time_until_obstacle);
          else
              ;// do nothing
      }

      // determine the directoin to go from cost
      double sum_l = 0.0;
      double sum_r = 0.0;
      double ave_l = 0.0;
      double ave_r = 0.0;
      for(int i = 0; i < times_until_obstacle_l.size(); i++)
          sum_l += times_until_obstacle_l[i];
      for(int i = 0; i < times_until_obstacle_r.size(); i++)
          sum_r += times_until_obstacle_r[i];
      ave_l = sum_l / times_until_obstacle_l.size();
      ave_r = sum_r / times_until_obstacle_r.size();
      ROS_DEBUG_NAMED ("top", "sum_l = %.2f, sum_r = %.2f", sum_l, sum_r);
      ROS_DEBUG_NAMED ("top", "size_l = %d, size_r = %d", (int)times_until_obstacle_l.size(), (int)times_until_obstacle_r.size());
      ROS_DEBUG_NAMED ("top", "ave_l = %.2f, ave_r = %.2f", ave_l, ave_r);

      double min_l = *min_element(times_until_obstacle_l.begin(), times_until_obstacle_l.end());
      double min_r = *min_element(times_until_obstacle_r.begin(), times_until_obstacle_r.end());
      ROS_INFO_NAMED ("top", "min_l = %.2f, min_r = %.2f", min_l, min_r);

      double z = angular_vel_steer_;
      //if(ave_l > ave_r)
      if(min_l < min_r)
          z *= -1;
      //else
      //    z = -0.5;

      // clear way
      //-- first steering
      twist.linear.x = linear_vel_steer_;
      twist.angular.z = z;
      moveSpacifiedLength(twist, duration_steer_);

      if(only_single_steering_) {
          //-- go straight
          twist.linear.x = linear_vel_forward_;
          twist.angular.z = 0.0;
          moveSpacifiedLength(twist, duration_forward_);

          //-- second steering
          twist.linear.x = linear_vel_steer_;
          twist.angular.z = -z;
          moveSpacifiedLength(twist, duration_steer_);
      }

      // stop
      d = 1.0;
      for (double t=0; t<d; t+=1/controller_frequency_) {
          pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, d-t));
          r.sleep();
      }

      //if(cnt == trial_times_)
      if(cnt == 1000)
      {
          ROS_INFO_NAMED ("top", "break after %d times recovery", cnt);
          break;
      }

      const  gm::Pose2D& current3 = getCurrentLocalPose();
      //max = 0.05;
      max = 0.1;
      min = -max;
      double max_time = 0;
      twist.linear.x = 3.0;
      for(double angle = min; angle < max; angle+=0.01)
      {
          twist.angular.z = angle;
          double time_until_obstacle = nonincreasingCostInterval(current3, twist);
          if(time_until_obstacle > max_time)
              max_time = time_until_obstacle;
      }

      if(max_time < 3)
      {
          ROS_INFO_NAMED ("top", "continue at (%.2f, %.2f, %.2f) for max_time %.2f seconds",
                          twist.linear.x, twist.linear.y, twist.angular.z, max_time);
          continue;
      }
      else
      {
          ROS_INFO_NAMED ("top", "break at (%.2f, %.2f, %.2f) for max_time %.2f seconds",
                          twist.linear.x, twist.linear.y, twist.angular.z, max_time);
          break;
      }
  }

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "**********Finish StepBackAndSteerRecovery!!**********");
  ROS_INFO_NAMED ("top", "*****************************************************");

}


} // namespace step_back_and_max_steer_recovery
