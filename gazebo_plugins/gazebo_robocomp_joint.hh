#ifndef GAZEBO_ROBOCOMP_JOINT_HH
#define GAZEBO_ROBOCOMP_JOINT_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <stdio.h>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    class GazeboRoboCompJoint : public ModelPlugin
    {
    public:
        // Constructor
        GazeboRoboCompJoint();

        // Destructor
        ~GazeboRoboCompJoint();

        // Load the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void SetVelocity(const double &_vel);

        void OnMsg(ConstVector3dPtr &_vel);

    private: 
        std::string topic_name_;

        // World name
        std::string world_name_;

        // Pointer to the World
        physics::WorldPtr world_;

        // Pointer to the model
        physics::ModelPtr model_;

        // Pointer to the joint
        physics::JointPtr joint_;

        // A simple PID controller

        common::PID pid_;

        // SDF root element
        sdf::ElementPtr sdf_;

        // Gazebo transport details
        gazebo::transport::NodePtr gazebo_node_;
        gazebo::transport::SubscriberPtr sub_;
        gazebo::transport::PublisherPtr pub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        gazebo::event::ConnectionPtr updateConnection_;

        int seed;
    };
}

#endif