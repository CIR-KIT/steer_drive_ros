
#include <angles/angles.h>

#include <urdf_parser/urdf_parser.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <pluginlib/class_list_macros.h>

#include <steer_bot_hardware_gazebo/steer_bot_hardware_gazebo.h>

namespace steer_bot_hardware_gazebo
{
  using namespace hardware_interface;

  SteerBotHardwareGazebo::SteerBotHardwareGazebo()
    : gazebo_ros_control::RobotHWSim()
  {}


  bool SteerBotHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;

    log_cnt_ = 0;

    ns_ = "steer_drive_controller/";//robot_namespace;
    nh_ = nh;

    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    this->CleanUp();
    this->GetJointNames(nh_);
    this->RegisterHardwareInterfaces();

#ifdef SPAWN_DEBUG
    // Cleanup
    sim_joints_.clear();
    jnt_pos_.clear();
    jnt_vel_.clear();
    jnt_eff_.clear();
    jnt_pos_cmd_.clear();

    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    std::vector<std::string> jnt_names;
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_names.push_back(sim_joints_[i]->GetName());
    }

    // Raw data
    jnt_pos_.resize(n_dof_);
    jnt_vel_.resize(n_dof_);
    jnt_eff_.resize(n_dof_);
    jnt_pos_cmd_.resize(n_dof_);

    // Hardware interfaces
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_state_interface_.registerHandle(
          JointStateHandle(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));

      jnt_pos_cmd_interface_.registerHandle(
          JointHandle(jnt_state_interface_.getHandle(jnt_names[i]), &jnt_pos_cmd_[i]));

      ROS_INFO_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_cmd_interface_);
#endif

#ifdef SPAWN_DEBUG
    // Position joint limits interface
    std::vector<std::string> cmd_handle_names = steer_jnt_pos_cmd_interface_.getNames();
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const std::string name = cmd_handle_names[i];

      // unless current handle is not pos interface for steer, skip
      if(name != virtual_steer_jnt_names_[INDEX_RIGHT] && name != virtual_steer_jnt_names_[INDEX_LEFT])
          continue;

      JointHandle cmd_handle = steer_jnt_pos_cmd_interface_.getHandle(name);

      using namespace joint_limits_interface;
      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
      JointLimits limits;
      SoftJointLimits soft_limits;
      if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
      {
        ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      }
      else
      {
        jnt_limits_interface_.registerHandle(
            PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

        ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
      }
    }
#endif

    // PID controllers for wheel
    const int virtual_jnt_cnt_ = virtual_wheel_jnt_names_.size();
    pids_.resize(virtual_jnt_cnt_);

    for (size_t i = 0; i < virtual_jnt_cnt_; ++i)
    {
      const std::string jnt_name = virtual_steer_jnt_names_[i];
      const ros::NodeHandle joint_nh(nh, "gains/" +  jnt_name);

      ROS_INFO_STREAM("Trying to set pid param of '" << jnt_name << " ' at PID proc in init()");
      if (!pids_[i].init(joint_nh))
      {
        ROS_INFO_STREAM("Faied to set pid param of '" << jnt_name << " ' at PID proc in init()");
        return false;
      }
      ROS_INFO_STREAM("Succeeded to set pid param of '" << jnt_name << " ' at PID proc in init()");
    }

    return true;
  }

  void SteerBotHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {

#ifdef SPAWN_DEBUG
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_pos_[i] += angles::shortest_angular_distance
          (jnt_pos_[i], sim_joints_[i]->GetAngle(0u).Radian());
      jnt_vel_[i] = sim_joints_[i]->GetVelocity(0u);
      jnt_eff_[i] = sim_joints_[i]->GetForce(0u);
    }

    for(int i = 0; i <  sim_joints_.size(); i++)
    {
        std::string gazebo_jnt_name;
        gazebo_jnt_name = sim_joints_[i]->GetName();
    }
#endif

    //ROS_INFO_STREAM("sim_joints_.size() = " << sim_joints_.size());

    for(int i = 0; i <  sim_joints_.size(); i++)
    {
        std::string gazebo_jnt_name;
        gazebo_jnt_name = sim_joints_[i]->GetName();
        //ROS_INFO_STREAM("CurrenReadingt gazebo joint '" << gazebo_jnt_name << " ' in readSim()");

        if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_RIGHT])
        {
            //ROS_INFO_STREAM("Reading gazebo joint '" << gazebo_jnt_name << " ' in readSim()");
            virtual_wheel_jnt_pos_[INDEX_RIGHT] = sim_joints_[i]->GetAngle(0u).Radian();
            virtual_wheel_jnt_vel_[INDEX_RIGHT] = sim_joints_[i]->GetVelocity(0u);
            virtual_wheel_jnt_eff_[INDEX_RIGHT] = sim_joints_[i]->GetForce(0u);
        }
        else if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_LEFT])
        {
            //ROS_INFO_STREAM("Reading gazebo joint '" << gazebo_jnt_name << " ' in readSim()");
            virtual_wheel_jnt_pos_[INDEX_LEFT] = sim_joints_[i]->GetAngle(0u).Radian();
            virtual_wheel_jnt_vel_[INDEX_LEFT] = sim_joints_[i]->GetVelocity(0u);
            virtual_wheel_jnt_eff_[INDEX_LEFT] = sim_joints_[i]->GetForce(0u);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT])
        {
            //ROS_INFO_STREAM("Reading gazebo joint '" << gazebo_jnt_name << " ' in readSim()");
            virtual_steer_jnt_pos_[INDEX_RIGHT] = sim_joints_[i]->GetAngle(0u).Radian();
            virtual_steer_jnt_vel_[INDEX_RIGHT] = sim_joints_[i]->GetVelocity(0u);
            virtual_steer_jnt_eff_[INDEX_RIGHT] = sim_joints_[i]->GetForce(0u);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
        {
            //ROS_INFO_STREAM("Reading gazebo joint '" << gazebo_jnt_name << " ' in readSim()");
            virtual_steer_jnt_pos_[INDEX_LEFT] = sim_joints_[i]->GetAngle(0u).Radian();
            virtual_steer_jnt_vel_[INDEX_LEFT] = sim_joints_[i]->GetVelocity(0u);
            virtual_steer_jnt_eff_[INDEX_LEFT] = sim_joints_[i]->GetForce(0u);
        }
        else
        {
            // do nothing
        }
    }
  }

  void SteerBotHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
    //jnt_limits_interface_.enforceLimits(period);
#ifdef SPAWN_DEBUG
    // Compute and send commands
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const double error = jnt_pos_cmd_[i] - jnt_pos_[i];
      //const double effort = pids_[i].computeCommand(error, period);
      const double effort = 0.;

      sim_joints_[i]->SetForce(0u, effort);
    }
#endif
    log_cnt_++;
    for(int i = 0; i <  sim_joints_.size(); i++)
    {
        std::string gazebo_jnt_name;
        gazebo_jnt_name = sim_joints_[i]->GetName();

        if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_RIGHT])
        {
            const double eff_cmd = ComputeEffCommandFromVelError(INDEX_RIGHT, period);
            sim_joints_[i]->SetForce(0u, eff_cmd);

            if(log_cnt_ % 500 == 0)
            {
                ROS_DEBUG_STREAM("wheel_jnt_vel_cmd_ = " << wheel_jnt_vel_cmd_);
                ROS_DEBUG_STREAM("virtual_wheel_jnt_vel_[INDEX_RIGHT] = " << virtual_wheel_jnt_vel_[INDEX_RIGHT]);
                ROS_DEBUG_STREAM("error[INDEX_RIGHT] " <<  wheel_jnt_vel_cmd_ - virtual_wheel_jnt_vel_[INDEX_RIGHT]);
                ROS_DEBUG_STREAM("command[INDEX_RIGHT] = " << eff_cmd);
            }
        }
        else if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_LEFT])
        {
            const double eff_cmd = ComputeEffCommandFromVelError(INDEX_LEFT, period);
            sim_joints_[i]->SetForce(0u, eff_cmd);

            if(log_cnt_ % 500 == 0)
            {
                ROS_DEBUG_STREAM("wheel_jnt_vel_cmd_ = " << wheel_jnt_vel_cmd_);
                ROS_DEBUG_STREAM("virtual_wheel_jnt_vel_[INDEX_LEFT] = " << virtual_wheel_jnt_vel_[INDEX_LEFT]);
                ROS_DEBUG_STREAM("error[INDEX_LEFT] " <<  wheel_jnt_vel_cmd_ - virtual_wheel_jnt_vel_[INDEX_LEFT]);
                ROS_DEBUG_STREAM("command[INDEX_LEFT] = " << eff_cmd);
            }
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT] || gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
        {
            sim_joints_[i]->SetAngle(0, steer_jnt_pos_cmd_);
        }
        else
        {
            // do nothing
        }
    }
  }

  void SteerBotHardwareGazebo::CleanUp()
  {
    // wheel joint names
    wheel_jnt_name_.empty();
    virtual_wheel_jnt_names_.clear();
    // rear wheel joint
    wheel_jnt_pos_ = 0;
    wheel_jnt_vel_ = 0;
    wheel_jnt_eff_ = 0;
    wheel_jnt_vel_cmd_ = 0;

    // steer joint names
    steer_jnt_name_.empty();
    virtual_steer_jnt_names_.clear();
    // front steer joint
    steer_jnt_pos_ = 0;
    steer_jnt_vel_ = 0;
    steer_jnt_eff_ = 0;
    steer_jnt_pos_cmd_ = 0;
  }

  void SteerBotHardwareGazebo::GetJointNames(ros::NodeHandle &_nh)
  {
    this->GetWheelJointNames(_nh);
    this->GetSteerJointNames(_nh);
  }

  void SteerBotHardwareGazebo::GetWheelJointNames(ros::NodeHandle &_nh)
  {
    // wheel joint to get linear command
    _nh.getParam(ns_ + "rear_wheel", wheel_jnt_name_);

    // virtual wheel joint for gazebo control
    _nh.getParam(ns_ + "gazebo/virtual_rear_wheel", virtual_wheel_jnt_names_);

    virtual_wheel_jnt_pos_.resize(virtual_wheel_jnt_names_.size());
    virtual_wheel_jnt_vel_.resize(virtual_wheel_jnt_names_.size());
    virtual_wheel_jnt_eff_.resize(virtual_wheel_jnt_names_.size());
  }

  void SteerBotHardwareGazebo::GetSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", steer_jnt_name_);

    // virtual steer joint for gazebo control
    _nh.getParam(ns_ + "gazebo/virtual_front_steer", virtual_steer_jnt_names_);

    virtual_steer_jnt_pos_.resize(virtual_steer_jnt_names_.size());
    virtual_steer_jnt_vel_.resize(virtual_steer_jnt_names_.size());
    virtual_steer_jnt_eff_.resize(virtual_steer_jnt_names_.size());
  }

  void SteerBotHardwareGazebo::RegisterHardwareInterfaces()
  {
    this->RegisterSteerInterface();
    this->RegisterWheelInterface();
  }

  void SteerBotHardwareGazebo::RegisterWheelInterface()
  {
    hardware_interface::JointStateHandle state_handle_steer(wheel_jnt_name_,
        &wheel_jnt_pos_,
        &wheel_jnt_vel_,
        &wheel_jnt_eff_);
    wheel_jnt_state_interface_.registerHandle(state_handle_steer);
    // joint velocity command
    hardware_interface::JointHandle vel_handle(wheel_jnt_state_interface_.getHandle(wheel_jnt_name_),
        &wheel_jnt_vel_cmd_);
    wheel_jnt_vel_cmd_interface_.registerHandle(vel_handle);

    ROS_INFO_STREAM("Registered joint '" << wheel_jnt_name_ << " ' in the VelocityJointInterface");

    // register mapped interface to ros_control
    registerInterface(&wheel_jnt_state_interface_);
    registerInterface(&wheel_jnt_vel_cmd_interface_);
  }

  void SteerBotHardwareGazebo::RegisterSteerInterface()
  {
    hardware_interface::JointStateHandle state_handle_wheel(steer_jnt_name_,
        &steer_jnt_pos_,
        &steer_jnt_vel_,
        &steer_jnt_eff_);
    steer_jnt_state_interface_.registerHandle(state_handle_wheel);

    // joint position command
    hardware_interface::JointHandle pos_handle(steer_jnt_state_interface_.getHandle(steer_jnt_name_),
        &steer_jnt_pos_cmd_);
    steer_jnt_pos_cmd_interface_.registerHandle(pos_handle);

    ROS_INFO_STREAM("Registered joint '" << steer_jnt_name_ << " ' in the PositionJointInterface");

    // register mapped interface to ros_control
    registerInterface(&steer_jnt_state_interface_);
    registerInterface(&steer_jnt_pos_cmd_interface_);
  }

  double SteerBotHardwareGazebo::ComputeEffCommandFromVelError(const int _index, ros::Duration _period)
  {
      const double vel_error = wheel_jnt_vel_cmd_ - virtual_wheel_jnt_vel_[INDEX_LEFT];
      const double command = pids_[_index].computeCommand(vel_error, _period);

      const double effort_limit = 10.0;
      const double effort = clamp(command,
                                  -effort_limit, effort_limit);
      return effort;
  }

} // namespace rosbook_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(steer_bot_hardware_gazebo::SteerBotHardwareGazebo, gazebo_ros_control::RobotHWSim)
