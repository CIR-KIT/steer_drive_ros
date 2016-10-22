
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
    , ns_("steer_bot_hardware_gazebo/")
    , log_cnt_(0)
  {}


  bool SteerBotHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;

    nh_ = nh;

    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    this->CleanUp();
    this->GetJointNames(nh_);
    this->RegisterHardwareInterfaces();

    nh_.param(ns_ + "enable_ackermann_link", true);
    ROS_INFO_STREAM("enable_ackermann_link = " << (enable_ackermann_link_ ? "true" : "false"));

#ifdef JOINT_LIMIT
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
    const int virtual_jnt_cnt_ = virtual_rear_wheel_jnt_names_.size();
    pids_.resize(virtual_jnt_cnt_);

    for (size_t i = 0; i < virtual_jnt_cnt_; ++i)
    {
      const std::string jnt_name = virtual_rear_wheel_jnt_names_[i];
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
    for(int i = 0; i <  sim_joints_.size(); i++)
    {
      std::string gazebo_jnt_name;
      gazebo_jnt_name = sim_joints_[i]->GetName();

      if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_rear_wheel_jnt_pos_, virtual_rear_wheel_jnt_vel_, virtual_rear_wheel_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_rear_wheel_jnt_pos_, virtual_rear_wheel_jnt_vel_, virtual_rear_wheel_jnt_eff_, INDEX_LEFT, i);
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_front_wheel_jnt_pos_, virtual_front_wheel_jnt_vel_, virtual_front_wheel_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_front_wheel_jnt_pos_, virtual_front_wheel_jnt_vel_, virtual_front_wheel_jnt_eff_, INDEX_LEFT, i);
      }
      else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_steer_jnt_pos_, virtual_steer_jnt_vel_, virtual_steer_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_steer_jnt_pos_, virtual_steer_jnt_vel_, virtual_steer_jnt_eff_, INDEX_LEFT, i);
      }
      else
      {
        // do nothing
      }
    }

    steer_jnt_pos_ = (virtual_steer_jnt_pos_[INDEX_RIGHT] + virtual_steer_jnt_pos_[INDEX_LEFT]) / virtual_steer_jnt_pos_.size();
    steer_jnt_vel_ = (virtual_steer_jnt_vel_[INDEX_RIGHT] + virtual_steer_jnt_vel_[INDEX_LEFT]) / virtual_steer_jnt_vel_.size();
    steer_jnt_eff_ = (virtual_steer_jnt_eff_[INDEX_RIGHT] + virtual_steer_jnt_eff_[INDEX_LEFT]) / virtual_steer_jnt_eff_.size();
  }

  void SteerBotHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
#ifdef JOINT_LIMIT
    jnt_limits_interface_.enforceLimits(period);
#endif
    log_cnt_++;
    for(int i = 0; i <  sim_joints_.size(); i++)
    {
      std::string gazebo_jnt_name;
      gazebo_jnt_name = sim_joints_[i]->GetName();

      if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_RIGHT])
      {
        const double eff_cmd = ComputeEffCommandFromVelError(INDEX_RIGHT, period);
        sim_joints_[i]->SetForce(0u, eff_cmd);

        if(log_cnt_ % 500 == 0)
        {
          ROS_DEBUG_STREAM("wheel_jnt_vel_cmd_ = " << wheel_jnt_vel_cmd_);
          ROS_DEBUG_STREAM("virtual_rear_wheel_jnt_vel_[INDEX_RIGHT] = " << virtual_rear_wheel_jnt_vel_[INDEX_RIGHT]);
          ROS_DEBUG_STREAM("error[INDEX_RIGHT] " <<  wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[INDEX_RIGHT]);
          ROS_DEBUG_STREAM("command[INDEX_RIGHT] = " << eff_cmd);
        }
      }
      else if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_LEFT])
      {
        const double eff_cmd = ComputeEffCommandFromVelError(INDEX_LEFT, period);
        sim_joints_[i]->SetForce(0u, eff_cmd);

        if(log_cnt_ % 500 == 0)
        {
          ROS_DEBUG_STREAM("wheel_jnt_vel_cmd_ = " << wheel_jnt_vel_cmd_);
          ROS_DEBUG_STREAM("virtual_rear_wheel_jnt_vel_[INDEX_LEFT] = " << virtual_rear_wheel_jnt_vel_[INDEX_LEFT]);
          ROS_DEBUG_STREAM("error[INDEX_LEFT] " <<  wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[INDEX_LEFT]);
          ROS_DEBUG_STREAM("command[INDEX_LEFT] = " << eff_cmd);
        }
      }
      else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT])
      {
        double pos_cmd = 0.0;
        if(enable_ackermann_link_)
        {
          double h = 0.79;
          double w = 0.5;
          pos_cmd = atan2(2*h*tan(steer_jnt_pos_cmd_), 2*h + w/2.0*tan(steer_jnt_pos_cmd_));
          ROS_DEBUG_STREAM("ackermann steer angle: " << pos_cmd << " at RIGHT");
        }
        else
        {
          pos_cmd = steer_jnt_pos_cmd_;
        }

        sim_joints_[i]->SetAngle(0, pos_cmd);
      }
      else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
      {
        double pos_cmd = 0.0;
        if(enable_ackermann_link_)
        {
          double h = 0.79;
          double w = 0.5;
          pos_cmd = atan2(2*h*tan(steer_jnt_pos_cmd_), 2*h - w/2.0*tan(steer_jnt_pos_cmd_));
          ROS_DEBUG_STREAM("ackermann steer angle: " << pos_cmd << " at LEFT");
        }
        else
        {
          pos_cmd = steer_jnt_pos_cmd_;
        }

        sim_joints_[i]->SetAngle(0, pos_cmd);
      }
      else
      {
        // do nothing
      }
    }
  }

  void SteerBotHardwareGazebo::CleanUp()
  {
    // wheel
    //-- wheel joint names
    wheel_jnt_name_.empty();
    virtual_rear_wheel_jnt_names_.clear();
    //-- actual rear wheel joint
    wheel_jnt_pos_ = 0;
    wheel_jnt_vel_ = 0;
    wheel_jnt_eff_ = 0;
    wheel_jnt_vel_cmd_ = 0;
    //-- virtual rear wheel joint
    virtual_rear_wheel_jnt_pos_.clear();
    virtual_rear_wheel_jnt_vel_.clear();
    virtual_rear_wheel_jnt_eff_.clear();
    virtual_rear_wheel_jnt_vel_cmd_.clear();
    //-- virtual front wheel joint
    virtual_front_wheel_jnt_pos_.clear();
    virtual_front_wheel_jnt_vel_.clear();
    virtual_front_wheel_jnt_eff_.clear();
    virtual_front_wheel_jnt_vel_cmd_.clear();

    // steer
    //-- steer joint names
    steer_jnt_name_.empty();
    virtual_steer_jnt_names_.clear();
    //-- front steer joint
    steer_jnt_pos_ = 0;
    steer_jnt_vel_ = 0;
    steer_jnt_eff_ = 0;
    steer_jnt_pos_cmd_ = 0;
    //-- virtual wheel joint
    virtual_steer_jnt_pos_.clear();
    virtual_steer_jnt_vel_.clear();
    virtual_steer_jnt_eff_.clear();
    virtual_steer_jnt_pos_cmd_.clear();
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
    _nh.getParam(ns_ + "virtual_rear_wheels", virtual_rear_wheel_jnt_names_);
    int dof = virtual_rear_wheel_jnt_names_.size();
    virtual_rear_wheel_jnt_pos_.resize(dof);
    virtual_rear_wheel_jnt_vel_.resize(dof);
    virtual_rear_wheel_jnt_eff_.resize(dof);
    virtual_rear_wheel_jnt_vel_cmd_.resize(dof);

    _nh.getParam(ns_ + "virtual_front_wheels", virtual_front_wheel_jnt_names_);
    dof = virtual_front_wheel_jnt_names_.size();
    virtual_front_wheel_jnt_pos_.resize(dof);
    virtual_front_wheel_jnt_vel_.resize(dof);
    virtual_front_wheel_jnt_eff_.resize(dof);
    virtual_front_wheel_jnt_vel_cmd_.resize(dof);

  }

  void SteerBotHardwareGazebo::GetSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", steer_jnt_name_);

    // virtual steer joint for gazebo control
    _nh.getParam(ns_ + "virtual_front_steers", virtual_steer_jnt_names_);

    const int dof = virtual_steer_jnt_names_.size();
    virtual_steer_jnt_pos_.resize(dof);
    virtual_steer_jnt_vel_.resize(dof);
    virtual_steer_jnt_eff_.resize(dof);
    virtual_steer_jnt_pos_cmd_.resize(dof);
  }

  void SteerBotHardwareGazebo::RegisterHardwareInterfaces()
  {
    this->RegisterSteerInterface();
    this->RegisterWheelInterface();

    // register mapped interface to ros_control
    registerInterface(&jnt_state_interface_);
    registerInterface(&wheel_jnt_vel_cmd_interface_);
    registerInterface(&steer_jnt_pos_cmd_interface_);
  }

  void SteerBotHardwareGazebo::RegisterInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff,  double& _jnt_cmd)
  {
    // register joint (both JointState and CommandJoint)
    this->RegisterJointStateInterfaceHandle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
    this->RegisterCommandJointInterfaceHandle(_jnt_state_interface, _jnt_cmd_interface,
                                              _jnt_name, _jnt_cmd);
  }

  void SteerBotHardwareGazebo::RegisterWheelInterface()
  {
    // actual wheel joints
    this->RegisterInterfaceHandles(
          jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
          wheel_jnt_name_, wheel_jnt_pos_, wheel_jnt_vel_, wheel_jnt_eff_, wheel_jnt_vel_cmd_);

    // virtual rear wheel joints
    for (int i = 0; i < virtual_rear_wheel_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
            virtual_rear_wheel_jnt_names_[i], virtual_rear_wheel_jnt_pos_[i], virtual_rear_wheel_jnt_vel_[i], virtual_rear_wheel_jnt_eff_[i], virtual_rear_wheel_jnt_vel_cmd_[i]);
    }
    // virtual front wheel joints
    for (int i = 0; i < virtual_front_wheel_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
            virtual_front_wheel_jnt_names_[i], virtual_front_wheel_jnt_pos_[i], virtual_front_wheel_jnt_vel_[i], virtual_front_wheel_jnt_eff_[i], virtual_front_wheel_jnt_vel_cmd_[i]);
    }
  }

  void SteerBotHardwareGazebo::RegisterSteerInterface()
  {
    // actual steer joints
    this->RegisterInterfaceHandles(
          jnt_state_interface_, steer_jnt_pos_cmd_interface_,
          steer_jnt_name_, steer_jnt_pos_, steer_jnt_vel_, steer_jnt_eff_, steer_jnt_pos_cmd_);

    // virtual steer joints
    for (int i = 0; i < virtual_steer_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, steer_jnt_pos_cmd_interface_,
            virtual_steer_jnt_names_[i], virtual_steer_jnt_pos_[i], virtual_steer_jnt_vel_[i], virtual_steer_jnt_eff_[i], virtual_steer_jnt_pos_cmd_[i]);
    }
  }

  void SteerBotHardwareGazebo::RegisterJointStateInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    hardware_interface::JointStateHandle state_handle(_jnt_name,
                                                      &_jnt_pos,
                                                      &_jnt_vel,
                                                      &_jnt_eff);
    _jnt_state_interface.registerHandle(state_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the JointStateInterface");
  }

  void SteerBotHardwareGazebo::RegisterCommandJointInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd)
  {
    // joint command
    hardware_interface::JointHandle _handle(_jnt_state_interface.getHandle(_jnt_name),
                                            &_jnt_cmd);
    _jnt_cmd_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the CommandJointInterface");
  }


  double SteerBotHardwareGazebo::ComputeEffCommandFromVelError(const int _index, ros::Duration _period)
  {
    double vel_error = wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[_index];
    ROS_DEBUG_STREAM("vel_error = " << vel_error);
    if(fabs(vel_error) < 0.1)
    {
      vel_error = 0.0;
      ROS_DEBUG_STREAM("too small. vel_error <- 0");
    }
    else
      ROS_DEBUG_STREAM("not small. ");

    const double command = pids_[_index].computeCommand(vel_error, _period);
    ROS_DEBUG_STREAM("command =" << command);

    const double effort_limit = 10.0;
    const double effort = clamp(command,
                                -effort_limit, effort_limit);
    return effort;
  }

  void SteerBotHardwareGazebo::GetCurrentState(std::vector<double>& _jnt_pos, std::vector<double>& _jnt_vel, std::vector<double>& _jnt_eff,
                                               const int _if_index, const int _sim_jnt_index)
  {
    _jnt_pos[_if_index] +=
        angles::shortest_angular_distance(_jnt_pos[_if_index], sim_joints_[_sim_jnt_index]->GetAngle(0u).Radian());
    _jnt_vel[_if_index] = sim_joints_[_sim_jnt_index]->GetVelocity(0u);
    _jnt_eff[_if_index] = sim_joints_[_sim_jnt_index]->GetForce(0u);
  }

} // namespace rosbook_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(steer_bot_hardware_gazebo::SteerBotHardwareGazebo, gazebo_ros_control::RobotHWSim)
