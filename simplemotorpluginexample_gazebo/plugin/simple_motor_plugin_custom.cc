/*
*  Name: simple_motor_plugin.cc
*  Author: Joseph Coombe
*  Date: 9/25/2017
*  Description:
*   A simple motor plugin for the Gazebo 7 robot simulator
*   Based on http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
*/

#ifndef _SIMPLE_MOTOR_PLUGIN_CUSTOM_HH_
#define _SIMPLE_MOTOR_PLUGIN_CUSTOM_HH_


#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <stdio.h>
#include <boost/bind.hpp>

//// ROS stuff ////
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{
  /// \brief A plugin to control a simple motor joint
  class SimpleMotorPluginCustom: public ModelPlugin
  {
    /// \brief Constructor
    public: SimpleMotorPluginCustom() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      std::cerr << "Loading SimpleMotorPluginCustom\n";
      // Store the model pointer for convenience.
      this->model =_model;

      // We are making an assumption about the model
      // having a continuous (revolving) "actuator" joint.
      // Check that the actuator element exists, then read the value.
      std::string actuator = ""; // default to empty string
      if (_sdf->HasElement("actuator")) {
        actuator = _sdf->Get<std::string>("actuator");
      }
      // TODO: print error message / throw exception if actuator was not in _sdf

      // Get the joint.
      this->joint = _model->GetJoint(actuator);
      // TODO: more error checking

      // Get the maxTorque from the sdf
      double maxTorque = 10.0;  // default torque
      if (_sdf->HasElement("maxTorque")) {
        maxTorque = _sdf->Get<double>("maxTorque");
      }

      // Setup a P-controller with a gain of 0.1
      // TODO: We'll want to replace this common::PID
      //       with our mockup HEBI controller algorithm
      this->pid = common::PID(80.0, 5.0, 40.0);

      // TODO: We'll want to support effort, velocity, and acceleration
      //       commands from ROS, but all of these will be converted to
      //       force commands in the plugin - just like the HEBI controller
      //       module.

      // Create the node
      this->cmd_vel_gaznode = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->cmd_vel_gaznode->Init(this->model->GetWorld()->GetName());
      #else
      this->cmd_vel_gaznode->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->cmd_vel_gaznode->Subscribe(topicName,
        &SimpleMotorPluginCustom::OnMsg, this);

      //// ROS stuff ////
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
        ros::init(argc, argv, "spinning_horizontal_arm",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->cmdVelRosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&SimpleMotorPluginCustom::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->cmdVelRosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&SimpleMotorPluginCustom::QueueThread, this));

      // Initialize update rate stuff
      this->update_rate_ = 100;
      if ( this->update_rate_ > 0.0 ) {
          this->update_period_ = 1.0 / this->update_rate_;
      } else {
          this->update_period_ = 0.0;
      }
      this->last_update_time_ = this->model->GetWorld()->GetSimTime();

      // Create our second ROS node.
      this->jointAngleRosNode.reset(new ros::NodeHandle("spinning_horizontal_arm"));
      //this->rosPub = this->jointAngleRosNode->advertise<sensor_msgs::JointState> ("joint_state",1000);
      this->rosPub2 = this->jointAngleRosNode->advertise<std_msgs::Float32>("joint_angle",1000);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                   boost::bind ( &SimpleMotorPluginCustom::OnUpdate, this, _1 ) );
    }

    public: void OnUpdate (const common::UpdateInfo & _info)
    {
       common::Time current_time = this->model->GetWorld()->GetSimTime();
       double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
       if ( seconds_since_last_update > update_period_ ) {
         //ros::Time current_time = ros::Time::now();
         //joint_state_.header.stamp = current_time;
         //joint_state_.name.resize (1);
         //joint_state_.position.resize (1);
         //math::Angle angle = this->joint->GetAngle(0);
         //joint_state_.name[0] = joint->GetName();
         //joint_state_.position[0] = angle.Radian() ;
         //rosPub.publish( joint_state_ );
         last_update_time_+= common::Time ( update_period_ );

         this->angle2.data = this->joint->GetAngle(0).Radian();
         rosPub2.publish(this->angle2);
       }

       if (seconds_since_last_update > 0) {
         double forceApplied = this->pid.Update(
                          this->joint->GetAngle(0).Radian() -
                          this->targetVel, seconds_since_last_update);
         this->joint->SetForce(0, forceApplied);
       }
    }

    /// \brief Set the joint's target velocity
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double & _vel)
    {
      // Update target velocity.
      this->targetVel = _vel;
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr & _msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the actuator joint.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->cmdVelRosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private:
      double targetVel;
      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

    /// \brief JointState sensor_msgs
    sensor_msgs::JointState joint_state_;

    /// \brief JointState std_msgs;
    std_msgs::Float32 angle2;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief Pointer to the update connection event
    private: event::ConnectionPtr updateConnection;

    /// \brief a PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport.
    private: transport::NodePtr cmd_vel_gaznode;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    //// ROS stuff ////
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> cmdVelRosNode;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> jointAngleRosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS publisher
    private: ros::Publisher rosPub;

    /// \brief A second ROS publisher
    private: ros::Publisher rosPub2;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SimpleMotorPluginCustom)

}
#endif
