/*
*  Name: boilerplate_motor_plugin.h
*  Author: Joseph Coombe
*  Date: 10/01/2017
*  Description:
*   A boilerplate motor plugin for the Gazebo 7 robot simulator
*   Used https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_skid_steer_drive.cpp
*   and http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
*   as references.
*   Also consult also see https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
*   for good use of the ros_plugins package - handles sdf element extraction, etc.
*/

// Plugin header file
#include <simplemotorpluginexample_gazebo/boilerplate_motor_plugin.h>

#include <stdio.h>
#include <boost/bind.hpp>
#include <thread>

// Gazebo dependencies
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ROS dependencies
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{

  /// \brief Constructor
  BoilerplateMotorPlugin::BoilerplateMotorPlugin() {}

  /// \brief Destructor
  BoilerplateMotorPlugin::~BoilerplateMotorPlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into a simulation (e.g. via URDF/SDF definition file)
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void BoilerplateMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::cerr << "Loading BoilerplateMotorPlugin\n";
    // Store the model pointer for convenience.
    this->model_ = _model;


    //// IMPORT PLUGIN PARAMETERS FROM SDF ////
    // Get the actuatorJoint from SDF.
    std::string joint_name = ""; // default to empty string
    if ( _sdf->HasElement("actuatorJoint") ) {
      joint_name = _sdf->Get<std::string>("actuatorJoint");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <actuatorJoint>!!!");
    }

    // Store the actuator joint pointer.
    this->joint_ = this->model_->GetJoint(joint_name);

    // Get the maxTorque from SDF.
    this->max_torque_ = 40.0;
    if ( _sdf->HasElement("maxTorque") ) {
      this->max_torque_ = _sdf->Get<double>("maxTorque");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <maxTorque>, defaults to \
      value \"%f\" (N-m)", this->max_torque_);
    }

    // Get position PID P-Gain from SDF.
    this->pos_pid_p_gain_ = 10.0; // default torque
    if ( _sdf->HasElement("posPidPGain") ) {
      this->pos_pid_p_gain_ = _sdf->Get<double>("posPidPGain");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <posPidPGain>, defaults to \
      value \"%f\" ", this->pos_pid_p_gain_);
    }

    // Get position PID I-Gain from SDF.
    this->pos_pid_i_gain_ = 1.0; // default torque
    if ( _sdf->HasElement("posPidIGain") ) {
      this->pos_pid_i_gain_ = _sdf->Get<double>("posPidIGain");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <posPidIGain>, defaults to \
      value \"%f\" ", this->pos_pid_i_gain_);
    }

    // Get position PID D-Gain from SDF.
    this->pos_pid_d_gain_ = 1.0; // default torque
    if ( _sdf->HasElement("posPidDGain") ) {
      this->pos_pid_d_gain_ = _sdf->Get<double>("posPidDGain");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <posPidDGain>, defaults to \
      value \"%f\" ", this->pos_pid_d_gain_);
    }

    // Get position PID I-Max from SDF.
    this->pos_pid_i_max_ = 0.0; // default torque
    if ( _sdf->HasElement("posPidIMax") ) {
      this->pos_pid_i_max_ = _sdf->Get<double>("posPidIMax");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <posPidIMax>, defaults to \
      value \"%f\" ", this->pos_pid_i_max_);
    }

    // Get position PID I-Min from SDF.
    this->pos_pid_i_min_ = 0.0; // default torque
    if ( _sdf->HasElement("posPidIMin") ) {
      this->pos_pid_i_min_ = _sdf->Get<double>("posPidIMin");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <posPidIMin>, defaults to \
      value \"%f\" ", this->pos_pid_i_min_);
    }

    // Get position PID Output-Max from SDF.
    // TODO:
    // Get position PID Output-Min from SDF.
    // TODO:

    // Get the initialPosition from the SDF.
    double initial_position = 0.0; // default position
    if ( _sdf->HasElement("initialPosition") ) {
      initial_position = _sdf->Get<double>("initialPosition");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <initialPosition>, defaults to \
      value \"%f\" (rad)", initial_position);
    }
    this->SetTargetPosition(initial_position); // set the initial joint position

    // Get the maxAngularVelocity from the SDF.
    // TODO

    // Get the cmdPosTopic from the SDF.
    std::string cmdPosTopic = "boilerplate_motor_plugin_test/pos_cmd";
    if ( _sdf->HasElement("cmdPosTopic") ) {
      cmdPosTopic = _sdf->Get<std::string>("cmdPosTopic");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <cmdPosTopic>!");
    }

    // Get the rosNodeHandle from the SDF.
    std::string rosNodeHandle = "";
    if ( _sdf->HasElement("rosNodeHandle") ) {
      rosNodeHandle = _sdf->Get<std::string>("rosNodeHandle");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing (optional) <rosNodeHandle>");
    }

    // Get the anglePosTopic from the SDF.
    std::string anglePosTopic = "joint_angle";
    if ( _sdf->HasElement("anglePosTopic") ) {
      anglePosTopic = _sdf->Get<std::string>("anglePosTopic");
    } else {
      ROS_WARN_NAMED("boilerplate_motor_plugin", "BoilerplateMotorPlugin missing <anglePosTopic>!");
    }

    //// ADD A JOINT CONTOLLER ////
    // TODO: We'll want to replace this common::PID
    //       with a custom HEBI controller class
    this->pid_ = common::PID(this->pos_pid_p_gain_, this->pos_pid_i_gain_, this->pos_pid_d_gain_, this->pos_pid_i_max_, this->pos_pid_i_min_); // pGain, iGain, dGain, iMax, iMin, cmdMax, cmdMin

    // TODO: We'll probably want to also support effort
    //       and velocity commands from the ROS interface
    //       which will be translated internally into appropriate
    //       force commands - just like the HEBI motor controller.


    //// SETUP GAZEBO MESSAGING ////
    // Create the node.
    this->gaz_node_ = transport::NodePtr(new transport::Node());
    #if GAZEBO_MAJOR_VERSION < 8
    this->gaz_node_->Init(this->model_->GetWorld()->GetName());
    #else
    this->gaz_node_->Init(this->model_->GetWorld()->Name());
    #endif

    // Create a topic name.
    std::string topic_name = "~/" + cmdPosTopic;

    // Subscribe to the topic and register a callback.
    this->sub_ = this->gaz_node_->Subscribe(topic_name,
      &BoilerplateMotorPlugin::OnMsg, this);


    //// SETUP ROS MESSAGING ////
    // Initialize ros, if it has not already been initialized.
    if ( !ros::isInitialized() ){
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, rosNodeHandle,
          ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. It behaves in a similar manner to the Gazebo node.
    if (rosNodeHandle != "") {
      this->ros_node_.reset(new ros::NodeHandle(rosNodeHandle) ) ;
    } else {
      this->ros_node_.reset(new ros::NodeHandle) ;
    }

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32>(
          "/" + cmdPosTopic,
          1,
          boost::bind(&BoilerplateMotorPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->ros_queue_);
    this->ros_sub_ = this->ros_node_->subscribe(so);

    this->ros_pub_ = this->ros_node_->advertise<std_msgs::Float32>(anglePosTopic, 1000);

    // Spin up the queue helper thread.
    this->ros_queue_thread_ =
      std::thread(std::bind(&BoilerplateMotorPlugin::QueueThread, this));

    // Initialize update rate stuff.
    this->update_rate_ = 100;
    if ( this->update_rate_ > 0.0 ) {
        this->update_period_ = 1.0 / this->update_rate_;
    } else {
        this->update_period_ = 0.0;
    }
    this->last_update_time_ = this->model_->GetWorld()->GetSimTime();


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin (
      boost::bind ( &BoilerplateMotorPlugin::OnUpdate, this, _1 ) );
  }

  /// \brief Function called at each simulation interation
  void BoilerplateMotorPlugin::OnUpdate(const common::UpdateInfo & _info)
  {
    common::Time current_time = this->model_->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - this->last_update_time_ ).Double();
    if ( seconds_since_last_update > this->update_period_ ) {
      this->last_update_time_+= common::Time ( this-> update_period_ );
      this->joint_angle_.data = this->joint_->GetAngle(0).Radian();
      this->ros_pub_.publish(this->joint_angle_);
    }
    if (seconds_since_last_update > 0) {
      // Calculate controller correction.
      this->target_torque_ = this->pid_.Update( // TODO: Replace with HEBI controller call
        this->joint_->GetAngle(0).Radian() -
        this->target_pos_, seconds_since_last_update);

      // Enforce max torque limits.
      if ( this->target_torque_ > this->max_torque_ ) {
        this->target_torque_ = this->max_torque_;
      } else if ( this->target_torque_ < -this->max_torque_ ) {
        this->target_torque_ = -this->max_torque_;
      }

      // Apply corrective torque to joint.
      this->joint_->SetForce(0, this->target_torque_);
    }
  }

  /// \brief Set the target position of the joint
  /// \param[in] _pos New target position
  void BoilerplateMotorPlugin::SetTargetPosition(const double & _pos)
  {
    // Update target position.
    this->target_pos_ = _pos;
  }

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.
  void BoilerplateMotorPlugin::OnMsg(ConstVector3dPtr & _msg)
  {
    this->SetTargetPosition(_msg->x());
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the actuator joint.
  void BoilerplateMotorPlugin::OnRosMsg(const std_msgs::Float32ConstPtr & _msg)
  {
      this->SetTargetPosition(_msg->data);
  }

  /// \brief ROS helper function that processes messages
  void BoilerplateMotorPlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node_->ok())
    {
      this->ros_queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BoilerplateMotorPlugin)
}
