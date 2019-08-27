/*
*  Name: boilerplate_motor_plugin.h
*  Author: Joseph Coombe
*  Date: 10/01/2017
*  Description:
*   A boilerplate motor plugin for the Gazebo 7 robot simulator
*/

#ifndef _BOILERPLATE_MOTOR_PLUGIN_HH_
#define _BOILERPLATE_MOTOR_PLUGIN_HH_

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

  class BoilerplateMotorPlugin: public ModelPlugin
  {

    public:
      BoilerplateMotorPlugin();
      ~BoilerplateMotorPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo & _info);

    private:
      void SetTargetPosition(const double & _pos);
      void OnMsg(ConstVector3dPtr & _msg);
      void OnRosMsg(const std_msgs::Float32ConstPtr & _msg);
      void QueueThread();

      double max_torque_;
      double pos_pid_p_gain_;
      double pos_pid_i_gain_;
      double pos_pid_d_gain_;
      double pos_pid_i_max_;
      double pos_pid_i_min_;
      double pos_pid_output_max_;
      double pos_pid_output_min_;

      double target_pos_;
      double target_torque_;

      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      sensor_msgs::JointState joint_state_;
      std_msgs::Float32 joint_angle_;

      /// \brief Pointer to the model.
      physics::ModelPtr model_;

      /// \brief Pointer to the joint.
      physics::JointPtr joint_;

      /// \brief Pointer to the update connection event
      event::ConnectionPtr update_connection_;

      /// \brief a PID controller for the joint.
      common::PID pid_;

      /// \brief Pointer to transport node.
      transport::NodePtr gaz_node_;

      /// \brief Pointer to a subscriber to a named topic.
      transport::SubscriberPtr sub_;

      //// ROS stuff ////
      /// \brief Pointer to a node used for ROS transport
      std::unique_ptr<ros::NodeHandle> ros_node_;

      /// \brief A ROS subscriber
      ros::Subscriber ros_sub_;

      /// \brief A ROS publisher
      ros::Publisher ros_pub_;

      /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue ros_queue_;

      /// \brief A thread the keeps running the rosQueue
      std::thread ros_queue_thread_;

  };

}


#endif /* _BOILERPLATE_MOTOR_PLUGIN_HH_ */
