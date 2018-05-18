#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include "gazebo_robocomp_laser.hh"

#include "raysensor.pb.h"
#include "Laser_msgs.pb.h"


namespace gazebo
{

GazeboRoboCompLaser::GazeboRoboCompLaser()
{
  this->seed = 0;
  this->topic_name_ = "/laser_scan_data_";
}

//////////////////////////////////////////////////
GazeboRoboCompLaser::~GazeboRoboCompLaser()
{
  // delete this->gazebo_node_;
}

//////////////////////////////////////////////////
void GazeboRoboCompLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    std::cerr << "Inside the LOAD" << std::endl;
    // load plugin
    RayPlugin::Load(_parent, _sdf);

    // Get the world name
    std::string world_name_ = _parent->WorldName();

    this->world_ = physics::get_world(world_name_);
    
    // save pointers
    this->sdf_ = _sdf;

    // GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;

    this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
        gzthrow("GazeboRoboCompLaser controller requires a Ray Sensor as its parent");

    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);
    
    this->topic_name_ = this->parent_ray_sensor_->Topic();

    std::cerr << "Topic Name: " << this->topic_name_ << std::endl;

    // Publisher to the topic
    // this->laser_scan_pub_ = this->gazebo_node_->Advertise<Laser_msgs::msgs::gazebo_robocomp_laser>(this->topic_name_);
    this->laser_scan_sub_ =
          this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
          &GazeboRoboCompLaser::OnScan, this);
}

///////////////////////////////////////////////////

void GazeboRoboCompLaser::OnNewLaserScans()
{
    
 /*   physics::MultiRayShapePtr laser = this->parent_ray_sensor_->LaserShape();
    // gzthrow("plugin is working");

    Laser_msgs::msgs::gazebo_robocomp_laser msg;
    gazebo::msgs::RaySensor* Ray;

    int sampleCount = 0;
    sampleCount = laser->GetSampleCount();

    std::cerr << "Sample count: " << sampleCount << std::endl;
    std::cerr << "Scan Resolution: " << laser->GetScanResolution() << std::endl;
    std::cerr << "Min Horizontal Angle: " << (laser->GetMinAngle()).Radian() << std::endl;
    std::cerr << "Max Horizontal Angle: " << (laser->GetMaxAngle()).Radian() << std::endl;
    std::cerr << "Vertical Samples: " << laser->GetVerticalSampleCount() << std::endl;
    std::cerr << "Vertical Resolution: " << laser->GetVerticalScanResolution() << std::endl;
    std::cerr << "Vertical Min Angle: " << (laser->GetVerticalMinAngle()).Radian() << std::endl;
    std::cerr << "Vertical Max Angle: " << (laser->GetVerticalMaxAngle()).Radian() << std::endl;
    std::cerr << "Range Min: " << laser->GetMinRange() << std::endl;
    std::cerr << "Range Max: " << laser->GetMaxRange() << std::endl;
    std::cerr << "Range Resolution: " << laser->GetResRange() << std::endl;
 

  // Ray->set_horizontal_samples(sampleCount);
  // Ray->set_horizontal_resolution(laser->GetScanResolution());
  // Ray->set_horizontal_min_angle((laser->GetMinAngle()).Radian());
  // Ray->set_horizontal_max_angle((laser->GetMaxAngle()).Radian());
  // Ray->set_vertical_samples(laser->GetVerticalSampleCount());
  // Ray->set_vertical_resolution(laser->GetVerticalScanResolution());
  // Ray->set_vertical_min_angle((laser->GetVerticalMinAngle()).Radian());
  // Ray->set_vertical_max_angle((laser->GetVerticalMaxAngle()).Radian());
  // Ray->set_range_min(laser->GetMinRange());
  // Ray->set_range_max(laser->GetMaxRange());
  // Ray->set_range_resolution(laser->GetResRange());
    
  // msg.set_allocated_laser(Ray);

  // this->laser_scan_pub_->Publish(msg); */
}

void GazeboRoboCompLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{
    // std::cerr << "Sample count: " << sampleCount << std::endl;
    // std::cerr << "Scan Resolution: " << laser->GetScanResolution() << std::endl;
    std::cerr << "Min Horizontal Angle: " << _msg->scan().angle_min() << std::endl;
    // std::cerr << "Max Horizontal Angle: " << (laser->GetMaxAngle()).Radian() << std::endl;
 
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompLaser)

}
