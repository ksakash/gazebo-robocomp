#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

#include "gazebo_robocomp_DiffDrive.hh"

namespace gazebo
{
// Register plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompDiffDrive)

/////////////////////////////////////////////////////
GazeboRoboCompDiffDrive::GazeboRoboCompDiffDrive()
{
}

//////////////////////////////////////////////////
GazeboRoboCompDiffDrive::~GazeboRoboCompDiffDrive()
{
}

//////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cerr << "DiffDrive Plugin Loaded!!!!" << std::endl;
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count,pDiffDrive plugin not loaded\n";
    return;
  }

  // Store the model pointer for convenience.
  this->model_ = _model;

  this->sdf_ = _sdf;

  std::cerr << "JointCount: " << model_->GetJointCount() << std::endl;

  this->world_name_ = this->model_->GetWorld()->GetName();

  this->left_joint_ = _model->GetJoints()[0];

  if (left_joint_ == NULL)
  {
    gzthrow("ERROR: Left Joint pointer is NULL");
    return;
  }
  else 
  {
    std::cerr << "Left Joint is working." << std::endl;
  }

  this->right_joint_ = _model->GetJoints()[0];

  if (right_joint_ == NULL)
  {
    gzthrow("ERROR: Right Joint pointer is NULL");
    return;
  }
  else
  {
    std::cerr << "Right Joint is working." << std::endl;
  }

  // Setup a P-controller, with a gain of 0.1.
  this->pid_ = common::PID(0.1, 0, 0);

  // Apply the p-controller to the joint.
  this->model_->GetJointController()->SetVelocityPID(this->right_joint_->GetScopedName(), this->pid_);
  this->model_->GetJointController()->SetVelocityPID(this->left_joint_->GetScopedName(), this->pid_);

  // Default to zero velocity
  double velocity = 0;
  double angular_vel = 0;

  // Check that the velocity element exists, then read the value
  if (sdf_->HasElement("velocity"))
    velocity = sdf_->Get<double>("velocity");
  
  // Create the Node
  this->gazebo_node_ = transport::NodePtr(new transport::Node());

  this->gazebo_node_->Init(world_name_);

  if (!this->sdf_->HasElement("topicName"))
  {
      std::cerr <<  "DiffDrive plugin missing <topicName>, defaults to /world" << std::endl;
      this->topic_name_ ="/my_robot";
  }
  else
      this->topic_name_ = this->sdf_->Get<std::string>("topicName");

  this->wheel_separation_ = this->left_joint_->GetAnchor(0).Distance(this->right_joint_->GetAnchor(0));

  // Subscribe to the topic, and register a callback
  this->sub_ = this->gazebo_node_->Subscribe(topic_name_, &GazeboRoboCompDiffDrive::OnMsg, this);
}

////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::SetVelocity(const double &_vel, const double &_ang_vel)
{
    // Set the joint's target velocity.

    std::cerr << "Setting linear velocity: " << _vel << " angular velocity: " << _ang_vel << std::endl;

    right_wheel_vel_ = _vel -  _ang_vel*wheel_separation_/2.0;
    left_wheel_vel_ = _vel + _ang_vel*wheel_separation_/2.0;

    this->model_->GetJointController()->SetVelocityTarget(this->right_joint_->GetScopedName(), 10);
    this->model_->GetJointController()->SetVelocityTarget(this->left_joint_->GetScopedName(), 10);
    
}

/////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::OnMsg(ConstVector3dPtr &_msg)
{
  this->SetVelocity(_msg->x(), _msg->y());
  std::cerr << "Got a command for linear velocity of " << _msg->x() << " and angular velocity of " << _msg->y() << std::endl;
}

}
