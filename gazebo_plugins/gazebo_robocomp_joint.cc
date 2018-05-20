#include "gazebo_robocomp_joint.hh"

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

#include <algorithm>
#include <string>
#include <assert.h>

// std::cerr << "Inside the definition file" << std::endl;

namespace gazebo
{
// A plugin to control a joint of a model

// Constructor
GazeboRoboCompJoint::GazeboRoboCompJoint() {
    std::cerr << "gazebo_robocomp_joint created" << std::endl;
}

GazeboRoboCompJoint::~GazeboRoboCompJoint() {}

// The load function is called by Gazebo when the plugin is  inserted into simulation
void GazeboRoboCompJoint::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Just output a message for now
    std::cerr << "\nThe velodyne plugin is attach to model[" << _model->GetName() << "]\n";

    // Safety check
    if (_model->GetJointCount() == 0)
    {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
    }

    // Store the model pointer for convenience.
    this->model_ = _model;

    this->world_name_ = _model->GetWorld()->GetName();

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint_ = _model->GetJoints()[0];

    // Setup a P-controller, with a gain of 0.1.
    this->pid_ = common::PID(0.1, 0, 0);

    // Apply the P-controller to the joint.
    this->model_->GetJointController()->SetVelocityPID(
            this->joint_->GetScopedName(), this->pid_);

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model_->GetJointController()->SetVelocityTarget(
            this->joint_->GetScopedName(), 10.0);

    // Default to zero velocity
    double velocity = 0;

    // Check that the velocity element exists, then read the value
    if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model_->GetJointController()->SetVelocityTarget(
            this->joint_->GetScopedName(), velocity);

    // Create the node
    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    if (_sdf->HasElement("topicName"))
    {
        this->topic_name_ = _sdf->Get<std::string>("topicName");
    }
    else
    {
        // Create a topic name
        topic_name_ = "~/" + this->model_->GetName() + "/vel_cmd";
    }

    // Subscribe to the topic, and register a callback
    this->sub_ = this->gazebo_node_->Subscribe(topic_name_, &GazeboRoboCompJoint::OnMsg, this);

    std::cerr << "The plugin is listening on '" << topic_name_ << "'" << "topic." << std::endl;

}

// Set the velocity of the joint
void GazeboRoboCompJoint::SetVelocity(const double &_vel)
{
    // Set the joint's target velocity.
    this->model_->GetJointController()->SetVelocityTarget(this->joint_->GetScopedName(), _vel);
    std::cerr << "The velocity of the joint is set to: " << _vel << std::endl;
}

// Handle incoming message
void GazeboRoboCompJoint::OnMsg(ConstVector3dPtr &_msg)
{
    this->SetVelocity(_msg->x());
}

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompJoint)
}
