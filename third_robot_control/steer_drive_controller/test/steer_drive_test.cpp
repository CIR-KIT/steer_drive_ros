#include<stdio.h>
#include<iostream>
#include<ros/ros.h>
#include<steer_drive_controller/steer_drive_controller.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
// gazebo
#include <control_toolbox/pid.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

int configureVelocityJointInterface(hardware_interface::VelocityJointInterface&);

class SteerBot:
        //public gazebo_ros_control::RobotHWSim
        public hardware_interface::RobotHW
{
    // methods
public:
    SteerBot(ros::NodeHandle &_nh)
        : ns_("steer_drive_controller/")
    {
        this->CleanUp();
        this->GetJointNames(_nh);
#ifdef MULTIPLE_JOINTS
        this->Resize();
#endif
        this->RegisterHardwareInterfaces();
    }

    bool initSim(const std::string& _robot_namespace,
      ros::NodeHandle _nh,
      gazebo::physics::ModelPtr _model,
      const urdf::Model* const _urdf_model,
      std::vector<transmission_interface::TransmissionInfo> _transmissions)
    {
        using gazebo::physics::JointPtr;

        this->CleanUp();

        // simulation joints
        sim_joints_ = _model->GetJoints();
        int n_dof = sim_joints_.size();

#ifdef MULTIPLE_JOINTS
        this->GetJointNames(_nh);
        this->Resize();
#endif
        this->RegisterHardwareInterfaces();
    }

    void readSim(ros::Time time, ros::Duration period)
    {

    }

    void writeSim(ros::Time time, ros::Duration period)
    {

    }

    void read()
    {
        std::ostringstream os;

#ifdef MULTIPLE_JOINTS
        for (unsigned int i = 0; i < cnt_wheel_joints_ - 1; ++i)
        {
            os << wheel_joint_vel_cmd_[i] << ", ";
        }
        os << wheel_joint_vel_cmd_[cnt_wheel_joints_ - 1];
#endif
        os << wheel_jnt_vel_cmd_;

        ROS_INFO_STREAM("Commands for wheel joints: " << os.str());
    }

    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    void write()
    {
        bool running_ = true;
        if (running_)
        {
#ifdef MULTIPLE_JOINTS
            for (unsigned int i = 0; i < cnt_wheel_joints_; ++i)
            {
                // Note that pos_[i] will be NaN for one more cycle after we start(),
                // but that is consistent with the knowledge we have about the state
                // of the robot.
                wheel_joint_pos_[i] += wheel_joint_vel_[i]*getPeriod().toSec(); // update position
                wheel_joint_vel_[i] = wheel_joint_vel_cmd_[i]; // might add smoothing here later
            }
#endif
            wheel_jnt_pos_ += wheel_jnt_vel_ * getPeriod().toSec(); // update position
            wheel_jnt_vel_ = wheel_jnt_vel_cmd_;
        }
#if 0
        else
        {
            std::fill_n(joint_pos_, cnt_wheel_joints_, std::numeric_limits<double>::quiet_NaN());
            std::fill_n(joint_vel_, cnt_wheel_joints_, std::numeric_limits<double>::quiet_NaN());
        }
#endif
    }
private:
    void CleanUp()
    {
#ifdef MULTIPLE_JOINTS
       // wheel joints
       wheel_joint_names_.clear();
       wheel_joint_pos_.clear();
       wheel_joint_vel_.clear();
       wheel_joint_eff_.clear();
       wheel_joint_vel_cmd_.clear();
#endif
       // rear wheel joint
        wheel_jnt_pos_ = 0;
        wheel_jnt_vel_ = 0;
        wheel_jnt_eff_ = 0;
        wheel_jnt_vel_cmd_ = 0;

#ifdef MULTIPLE_JOINTS
       // steer joints
       steer_joint_names_.clear();
       steer_joint_pos_.clear();
       steer_joint_vel_.clear();
       steer_joint_eff_.clear();
       steer_joint_vel_cmd_.clear();
#endif
       // front steer joint
       steer_jnt_pos_ = 0;
       steer_jnt_vel_ = 0;
       steer_jnt_eff_ = 0;
       steer_jnt_pos_cmd_ = 0;

    }

#ifdef MULTIPLE_JOINTS
    void Resize()
    {
       // wheel joints
       wheel_joint_pos_.resize(cnt_wheel_joints_);
       wheel_joint_vel_.resize(cnt_wheel_joints_);
       wheel_joint_eff_.resize(cnt_wheel_joints_);
       wheel_joint_vel_cmd_.resize(cnt_wheel_joints_);

       // steer joints
       steer_joint_pos_.resize(cnt_steer_joints_);
       steer_joint_vel_.resize(cnt_steer_joints_);
       steer_joint_eff_.resize(cnt_steer_joints_);
       steer_joint_vel_cmd_.resize(cnt_steer_joints_);
    }
#endif
    void GetJointNames(ros::NodeHandle &_nh)
    {
#ifdef MULTIPLE_JOINTS
        this->GetWheelJointNames(_nh);
        this->GetSteerJointNames(_nh);
#endif

        _nh.getParam(ns_ + "rear_wheel", wheel_jnt_name_);
        _nh.getParam(ns_ + "front_steer", steer_jnt_name_);

    }

#ifdef MULTIPLE_JOINTS
    void GetWheelJointNames(ros::NodeHandle &_nh)
    {
        // wheel names
        std::vector<std::string> right_wheel_joint_names;
        std::vector<std::string> left_wheel_joint_names;

        _nh.getParam(ns_ + "right_wheel", right_wheel_joint_names);
        _nh.getParam(ns_ + "left_wheel", left_wheel_joint_names);

        wheel_joint_names_ = right_wheel_joint_names;
        std::copy(left_wheel_joint_names.begin(), left_wheel_joint_names.end(),
                  std::back_inserter(wheel_joint_names_));
        cnt_wheel_joints_ = wheel_joint_names_.size();

        // rear wheel name
        std::string wheel_joint_name;
        /*
        _nh.getParam(ns_ + "rear_wheel", wheel_joint_name);
        wheel_joint_name_ = wheel_joint_name;
        */
    }

    void GetSteerJointNames(ros::NodeHandle &_nh)
    {
        // steer names
        std::vector<std::string> right_steer_joint_names;
        std::vector<std::string> left_steer_joint_names;

        _nh.getParam(ns_ + "right_steer", right_steer_joint_names);
        _nh.getParam(ns_ + "left_steer", left_steer_joint_names);

        steer_joint_names_ = right_steer_joint_names;
        std::copy(left_steer_joint_names.begin(), left_steer_joint_names.end(),
                  std::back_inserter(steer_joint_names_));
        cnt_steer_joints_ = steer_joint_names_.size();

        // front steer name
        std::string steer_joint_name;
        /*
        _nh.getParam(ns_ + "front_steer", steer_joint_name);
        wheel_joint_name_ = steer_joint_name;
        */
    }
#endif

    void RegisterHardwareInterfaces()
    {
#ifdef MULTIPLE_JOINTS
        this->RegisterSteerInterfaces();
#endif
        this->RegisterSteerInterface();

#ifdef MULTIPLE_JOINTS
        this->RegisterWheelInterfaces();
#endif
        this->RegisterWheelInterface();
    }

    void RegisterWheelInterface()
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

    void RegisterWheelInterfaces()
    {
        // map members to hardware interfaces
        for (size_t i = 0; i < cnt_wheel_joints_; ++i) {
            // joint states
            hardware_interface::JointStateHandle state_handle(wheel_joint_names_[i],
                                                              &wheel_joint_pos_[i],
                                                              &wheel_joint_vel_[i],
                                                              &wheel_joint_eff_[i]);
            wheel_joint_state_interface_.registerHandle(state_handle);

            // joint velocity command
            hardware_interface::JointHandle vel_handle(wheel_joint_state_interface_.getHandle(wheel_joint_names_[i]),
                                                       &wheel_joint_vel_cmd_[i]);
            wheel_vel_joint_interface_.registerHandle(vel_handle);

            ROS_DEBUG_STREAM("Registered joint '" << wheel_joint_names_[i] << " ' in the VelocityJointInterface");
        }

        // register mapped interface to ros_control
        registerInterface(&wheel_joint_state_interface_);
        registerInterface(&wheel_vel_joint_interface_);
    }

    void RegisterSteerInterface()
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

    void RegisterSteerInterfaces()
    {
        // map members to hardware interfaces
        for (size_t i = 0; i < cnt_steer_joints_; ++i) {
            // joint states
            hardware_interface::JointStateHandle state_handle(steer_joint_names_[i],
                                                              &steer_joint_pos_[i],
                                                              &steer_joint_vel_[i],
                                                              &steer_joint_eff_[i]);
            steer_joint_state_interface_.registerHandle(state_handle);

            // joint position command
            hardware_interface::JointHandle pos_handle(steer_joint_state_interface_.getHandle(steer_joint_names_[i]),
                                                       &steer_joint_vel_cmd_[i]);
            steer_pos_joint_interface_.registerHandle(pos_handle);

            ROS_DEBUG_STREAM("Registered joint '" << steer_joint_names_[i] << " ' in the PositionJointInterface");
        }

        // register mapped interface to ros_control
        registerInterface(&steer_joint_state_interface_);
        registerInterface(&steer_pos_joint_interface_);
    }

    // member variables
public:
    hardware_interface::VelocityJointInterface wheel_vel_joint_interface_;
    hardware_interface::PositionJointInterface steer_pos_joint_interface_;

private:
    ros::NodeHandle nh_;

    std::string ns_;
    //
    std::vector<std::string> wheel_joint_names_;
    std::vector<std::string> steer_joint_names_;
    std::string wheel_jnt_name_;
    std::string steer_jnt_name_;

    // interface variables
    //-- wheel
    int cnt_wheel_joints_;
    std::vector<double> wheel_joint_pos_;
    std::vector<double> wheel_joint_vel_;
    std::vector<double> wheel_joint_eff_;
    std::vector<double> wheel_joint_vel_cmd_;
    //-- rear wheel
    double wheel_jnt_pos_;
    double wheel_jnt_vel_;
    double wheel_jnt_eff_;
    double wheel_jnt_vel_cmd_;

    //-- steer
    int cnt_steer_joints_;
    std::vector<double> steer_joint_pos_;
    std::vector<double> steer_joint_vel_;
    std::vector<double> steer_joint_eff_;
    std::vector<double> steer_joint_vel_cmd_;
    //-- front steer
    double steer_jnt_pos_;
    double steer_jnt_vel_;
    double steer_jnt_eff_;
    double steer_jnt_pos_cmd_;

    //
    hardware_interface::JointStateInterface wheel_joint_state_interface_;
    hardware_interface::JointStateInterface steer_joint_state_interface_;

    // gazebo
    std::vector<gazebo::physics::JointPtr> sim_joints_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steer_drive_test");
    ros::NodeHandle nh_main;
    ros::NodeHandle nh_control;

    SteerBot* steer_drive_bot = new SteerBot(nh_control);

    steer_drive_controller::SteerDriveController rb_controller;
#if 0
    bool rt_code = rb_controller.init(//&steer_drive_bot->wheel_vel_joint_interface_,
                                      steer_drive_bot,
                                      nh_main, nh_control);
#endif
    std::set<std::string> claimed_resources; // Gets populated during initRequest call
    bool rt_code = rb_controller.init(steer_drive_bot, nh_main, nh_control);//, claimed_resources);

    ros::Rate rate(100);
    ros::Time last_time = ros::Time::now();

    while(nh_main.ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - last_time;
        last_time = now;

        steer_drive_bot->read();
        rb_controller.update(now, period);
        steer_drive_bot->write();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
