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
#include "diffdrive_state.pb.h"
#include "diffdrive.proto.pb.h"

using namespace std;

namespace gazebo
{
// Register plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompDiffDrive)

GazeboRoboCompDiffDrive::GazeboRoboCompDiffDrive()
{
}

GazeboRoboCompDiffDrive::~GazeboRoboCompDiffDrive()
{
}

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
      this->sub_topic_name ="/my_robot";
  }
  else
      this->sub_topic_name = this->sdf_->Get<std::string>("topicName");

  this->wheel_separation_ = this->left_joint_->GetAnchor(0).Distance(this->right_joint_->GetAnchor(0));

  pub_topic_name_ = "/diffdrive/data"

  // Subscribe to the topic, and register a callback
  this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &GazeboRoboCompDiffDrive::OnMsg, this);
  this->pub_ = this->gazebo_node_->Advertise<gazebo::msgs::Pose>(pub_topic_name_);
  // this->diffdrive_pub_ = this->gazebo_mode_->Advertise<gazebo::msgs::>(diffdrive_state_topic_name_);

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRoboCompDiffDrive::OnUpdate, this ) );
}
////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::SetVelocity(const double &_vel, const double &_ang_vel)
{
    // Set the joint's target velocity.

    std::cerr << "Setting linear velocity: " << _vel << " angular velocity: " << _ang_vel << std::endl;

    right_wheel_vel_ = _vel -  _ang_vel*wheel_separation_/2.0;
    left_wheel_vel_ = _vel + _ang_vel*wheel_separation_/2.0;

    this->model_->GetJointController()->SetVelocityTarget(this->right_joint_->GetScopedName(), right_wheel_vel);
    this->model_->GetJointController()->SetVelocityTarget(this->left_joint_->GetScopedName(), left_wheel_vel);  
}

/////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::OnMsg(ConstDiffDriveCmdPtr &_msg)
{
  this->SetVelocity(_msg->linear_vel(), _msg->angular_pos());
  std::cerr << "Got a command for linear velocity of " << _msg->x() << " and angular velocity of " << _msg->y() << std::endl;
}

void GazeboRoboCompDiffDrive::OnUpdate() 
{
  ignition::math::Pose3d base_pose_ = model_->WorldPose();
  ignition::math::Vector3d base_lin_vel_ =model_->WorldLinearVel();
  ignition::math::Vector3d base_lin_accln = model_->WorldLinearAccel();
  ignition::math::Vector3d base_ang_vel = _model->WorldAngularVel();
  ignition::math::Vector3d base_ang_accln = _model->WorldAngularAccel();

  gazebo::msgs::DiffDriveState msg;

  deque<gazebo::msgs::Vector3dPtr> states;
  gazebo::msgs::PosePtr pose_;

  pose_ = msg.mutable_pose();
  states.push_back(msg.mutable_angVel());
  states.push_back(msg.mutable_angAccln());
  states.push_back(msg.mutable_linVel());
  states.push_back(msg.mutable_linAccln());
  
  deque<gazebo::msgs::Vector3dPtr>::iterator it;

  it = states.begin();
  (*it)->set_x() = base_ang_vel.X();
  (*it)->set_y() = base_ang_vel.Y();
  (*it)->set_z() = base_ang_vel.Z();
  
  i++;

  (*it)->set_x() = base_ang_accln.X();
  (*it)->set_y() = base_ang_accln.Y();
  (*it)->set_z() = base_ang_accln.Z();
  
  i++;

  (*it)->set_x() = base_lin_vel.X();
  (*it)->set_y() = base_lin_vel.Y();
  (*it)->set_z() = base_lin_vel.Z();
  
  i++;

  (*it)->set_x() = base_lin_accln.X();
  (*it)->set_y() = base_lin_accln.Y();
  (*it)->set_z() = base_lin_accln.Z();

  gazebo::msgs::Vector3dPtr pos_;
  gazebo::msgs::QuaternionPtr rot_;

  pos_ = pose_->mutable_rot();
  rot_ = pose_->mutable_pos();

  pos_->set_x() = base_pose_->pos().X();
  pos_->set_y() = base_pose_->pos().Y();
  pos_->set_z() = base_pose_->pos().Z();

  rot_->set_w() = base_pose_->rot().W();
  rot_->set_x() = base_pose_->rot().X();
  rot_->set_y() = base_pose_->rot().Y();
  rot_->set_z() = base_pose_->rot().Z();  

  pub_->Publish(msg);
}

}
