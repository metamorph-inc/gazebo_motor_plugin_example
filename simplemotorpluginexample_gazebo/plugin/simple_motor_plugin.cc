/*
*  Name: simple_motor_plugin.cc
*  Author: Joseph Coombe
*  Date: 9/25/2017
*  Description:
*   A simple motor plugin for the Gazebo 7 robot simulator
*   Based on http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
*/

#ifndef _SIMPLE_MOTOR_PLUGIN_HH_
#define _SIMPLE_MOTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//// ROS stuff ////
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a simple motor joint
  class SimpleMotorPlugin: public ModelPlugin
  {
    /// \brief Constructor
    public: SimpleMotorPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "\n The model does not have any joints! \
          SimpleMotorPlugin not loaded\n";
        return;
      }
      else
      {
        // Just outputs a message for now
        std::cerr << "\n The simple motor plugin is attached to model["
          << _model->GetName() << "]\n";
        // ^ Occasionaly generates the following message in roslaunch terminal:
        //  "The simple motor plugin is attached to model[spinning_horizontal_arm]
        //  Unhandled exception in thread started by
        //  sys.excepthook is missing
        //  lost sys.stderr"
      }

      // Store the model pointer for convenience.
      this->model =_model;

      // default to empty string
      std::string actuator = "";

      // We are making an assumption about the model
      // having a continuous (revolving) "actuator" joint.
      // Check that the actuator element exists, then read the value.
      if (_sdf->HasElement("actuator")) {
          actuator = _sdf->Get<std::string>("actuator");
      }
      // TODO: print error message / throw exception if actuator was not in _sdf

      // Get the joint.
      this->joint = _model->GetJoint(actuator);
      // TODO: more error checking

      // Setup a P-controller with a gain of 0.1
      // TODO: We'll want to replace this common::PID
      //       with our mockup HEBI controller algorithm
      this->pid = common::PID(0.5, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
        this->joint->GetScopedName(), this->pid);

      // TODO: We'll want to support effort, velocity, and acceleration
      //       commands from ROS, but all of these will be converted to
      //       force commands in the plugin - just like the HEBI controller
      //       module.

      // default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value.
      if (_sdf->HasElement("velocity")) {
        velocity = _sdf->Get<double>("velocity");
      }

      // Set initial velocity
      this->SetVelocity(velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
        &SimpleMotorPlugin::OnMsg, this);

      //// ROS stuff ////
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&SimpleMotorPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&SimpleMotorPlugin::QueueThread, this));
    }

    /// \brief Set the velocity of the joint
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double & _vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), _vel);
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
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief a PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport.
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    //// ROS stuff ////
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SimpleMotorPlugin)

}
#endif
