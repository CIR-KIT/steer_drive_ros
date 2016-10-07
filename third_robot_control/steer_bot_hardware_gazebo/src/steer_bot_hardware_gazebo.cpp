
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

    ns_ = "steer_drive_controller/";//robot_namespace;
    nh_ = nh;

    this->CleanUp();
    this->GetJointNames(nh_);
    this->RegisterHardwareInterfaces();

#ifdef STEER_BOT_MODIFY
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

        ROS_DEBUG_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_cmd_interface_);
#endif

    // Position joint limits interface
    std::vector<std::string> cmd_handle_names = jnt_pos_cmd_interface_.getNames();
    for (size_t i = 0; i < n_dof_; ++i)
    {
        const std::string name = cmd_handle_names[i];
        JointHandle cmd_handle = jnt_pos_cmd_interface_.getHandle(name);

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
#ifdef STEER_BOT_MODIFY
    // PID controllers
    pids_.resize(n_dof_);
    for (size_t i = 0; i < n_dof_; ++i)
    {
        ros::NodeHandle joint_nh(nh, "gains/" + jnt_names[i]);

        if (!pids_[i].init(joint_nh))
        {
            return false;
        }
    }
#endif

    return true;
}

void SteerBotHardwareGazebo::readSim(ros::Time time, ros::Duration period)
{

#ifdef STEER_BOT_MODIFY
    for (size_t i = 0; i < n_dof_; ++i)
    {
        jnt_pos_[i] += angles::shortest_angular_distance
                (jnt_pos_[i], sim_joints_[i]->GetAngle(0u).Radian());
        jnt_vel_[i] = sim_joints_[i]->GetVelocity(0u);
        jnt_eff_[i] = sim_joints_[i]->GetForce(0u);
    }
#endif

    double wheel_pos_sum = 0.;
    double wheel_vel_sum = 0.;
    double wheel_eff_sum = 0.;

    double steer_pos_sum = 0.;
    double steer_vel_sum = 0.;
    double steer_eff_sum = 0.;

    for(int i = 0; i <  sim_joints_.size(); i++)
    {
        std::string gazebo_jnt_name;
        gazebo_jnt_name = sim_joints_[i]->GetName();

        if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_RIGHT])
        {
            wheel_pos_sum += sim_joints_[i]->GetAngle(0u).Radian();
            wheel_vel_sum += sim_joints_[i]->GetVelocity(0u);
            wheel_eff_sum += sim_joints_[i]->GetForce(0u);
        }

        else if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_LEFT])
        {
            wheel_pos_sum += sim_joints_[i]->GetAngle(0u).Radian();
            wheel_vel_sum += sim_joints_[i]->GetVelocity(0u);
            wheel_eff_sum += sim_joints_[i]->GetForce(0u);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT])
        {
            steer_pos_sum += sim_joints_[i]->GetAngle(0u).Radian();
            steer_vel_sum += sim_joints_[i]->GetVelocity(0u);
            steer_eff_sum += sim_joints_[i]->GetForce(0u);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
        {
            steer_pos_sum += sim_joints_[i]->GetAngle(0u).Radian();
            steer_vel_sum += sim_joints_[i]->GetVelocity(0u);
            steer_eff_sum += sim_joints_[i]->GetForce(0u);
        }
        else
        {
            // do nothing
        }

    }

    // rear wheel
    //-- get average
    double wheel_pos_ave = wheel_pos_sum / (double)virtual_wheel_jnt_names_.size();
    double wheel_vel_ave = wheel_vel_sum / (double)virtual_wheel_jnt_names_.size();
    double wheel_eff_ave = wheel_eff_sum / (double)virtual_wheel_jnt_names_.size();
    //-- set current state
    wheel_jnt_pos_ += angles::shortest_angular_distance(wheel_jnt_pos_, wheel_pos_ave);
    wheel_jnt_vel_ = wheel_vel_ave;
    wheel_jnt_eff_ = wheel_eff_ave;

    // front steer
    //-- get average
    double steer_pos_ave = steer_pos_sum / (double)virtual_steer_jnt_names_.size();
    double steer_vel_ave = steer_vel_sum / (double)virtual_steer_jnt_names_.size();
    double steer_eff_ave = steer_eff_sum / (double)virtual_steer_jnt_names_.size();
    //-- set current state
    steer_jnt_pos_ += angles::shortest_angular_distance(steer_jnt_pos_, steer_pos_ave);
    steer_jnt_vel_ = steer_vel_ave;
    steer_jnt_eff_ = steer_eff_ave;

}

void SteerBotHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
{
    // Enforce joint limits
    jnt_limits_interface_.enforceLimits(period);

#ifdef STEER_BOT_MODIFY
    // Compute and send commands
    for (size_t i = 0; i < n_dof_; ++i)
    {
        const double error = jnt_pos_cmd_[i] - jnt_pos_[i];
        const double effort = pids_[i].computeCommand(error, period);

        sim_joints_[i]->SetForce(0u, effort);
    }
#endif
    wheel_jnt_vel_ = wheel_jnt_vel_cmd_;
    steer_jnt_pos_ = steer_jnt_pos_cmd_;

    for(int i = 0; i <  sim_joints_.size(); i++)
    {
        std::string gazebo_jnt_name;
        gazebo_jnt_name = sim_joints_[i]->GetName();

        if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_RIGHT])
        {
            sim_joints_[i]->SetVelocity(0, wheel_jnt_vel_cmd_);
        }
        else if(gazebo_jnt_name == virtual_wheel_jnt_names_[INDEX_LEFT])
        {
            sim_joints_[i]->SetVelocity(0, -1 * wheel_jnt_vel_cmd_);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_RIGHT])
        {
            sim_joints_[i]->SetAngle(0, steer_jnt_pos_cmd_);
        }
        else if(gazebo_jnt_name == virtual_steer_jnt_names_[INDEX_LEFT])
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
    std::vector<std::string> virtual_wheel_jnt_names;

    _nh.getParam(ns_ + "gazebo/virtual_rear_wheel", virtual_wheel_jnt_names);

    virtual_wheel_jnt_names_ = virtual_wheel_jnt_names;
}

void SteerBotHardwareGazebo::GetSteerJointNames(ros::NodeHandle &_nh)
{
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", steer_jnt_name_);

    // virtual steer joint for gazebo control
    std::vector<std::string> virtual_steer_jnt_names;

    _nh.getParam(ns_ + "gazebo/virtual_front_steer", virtual_steer_jnt_names);

    virtual_steer_jnt_names_ = virtual_steer_jnt_names;
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
    wheel_joint_state_interface_.registerHandle(state_handle_steer);
    // joint velocity command
    hardware_interface::JointHandle vel_handle(wheel_joint_state_interface_.getHandle(wheel_jnt_name_),
                                               &wheel_jnt_vel_cmd_);
    wheel_vel_joint_interface_.registerHandle(vel_handle);

    ROS_DEBUG_STREAM("Registered joint '" << wheel_jnt_name_ << " ' in the VelocityJointInterface");

    // register mapped interface to ros_control
    registerInterface(&wheel_joint_state_interface_);
    registerInterface(&wheel_vel_joint_interface_);
}

void SteerBotHardwareGazebo::RegisterSteerInterface()
{
    hardware_interface::JointStateHandle state_handle_wheel(steer_jnt_name_,
                                                            &steer_jnt_pos_,
                                                            &steer_jnt_vel_,
                                                            &steer_jnt_eff_);
    steer_joint_state_interface_.registerHandle(state_handle_wheel);

    // joint position command
    hardware_interface::JointHandle pos_handle(steer_joint_state_interface_.getHandle(steer_jnt_name_),
                                               &steer_jnt_pos_cmd_);
    steer_pos_joint_interface_.registerHandle(pos_handle);

    ROS_DEBUG_STREAM("Registered joint '" << steer_jnt_name_ << " ' in the PositionJointInterface");

    // register mapped interface to ros_control
    registerInterface(&steer_joint_state_interface_);
    registerInterface(&steer_pos_joint_interface_);
}

} // namespace steer_bot_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(steer_bot_hardware_gazebo::SteerBotHardwareGazebo, gazebo_ros_control::RobotHWSim)
