#ifndef GAZEBO_ROBOCOMP_JOINT_HH
#define GAZEBO_ROBOCOMP_JOINT_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

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

        void OnMsg(const double &_vel);

        private: std::string topic_name_;

        // World name
        private: std::string world_name_;

        // Pointer to the World
        private: physics::WorldPtr world_;

        // Pointer to the model
        private: physics::ModelPtr model_;

        // Pointer to the joint
        private: physics::JointPtr joint_;

        // A simple PID controller

        private: common::PID pid_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // Gazebo transport details
        private: gazebo::transport::NodePtr gazebo_node_;
        private: gazebo::transport::SubscriberPtr sub_;
        private: gazebo::transport::PublisherPtr pub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: gazebo::event::ConnectionPtr updateConnection_;

        private: int seed;
    };
}
#endif
