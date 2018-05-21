#include "gazebo_robocomp_camera.hh"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gazebo
{
  GazeboRoboCompCamera::GazeboRoboCompCamera()
  {
    this->seed = 0;
  }

  GazeboRoboCompCamera::~GazeboRoboCompCamera() {}

  void GazeboRoboCompCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    CameraPlugin::Load(_parent,_sdf);
    std::cout << "Load: " << " " <<  this->parentSensor->Camera()->Name()<< std::endl;

    this->parent_sensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->parent_sensor_->WorldName());
  }

  void GazeboRoboCompCamera::OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
  {
    if (seed == 0)
    {
      image_.create(_height, _width, CV_8UC3);
    }
    memcpy((unsigned char *) image_.data, &(_image[0]), _width*_height * 3);
    cv::imshow("Display Window", image_);
    std::cerr << "Showing Image" << std::endl;

  }

  void OnMsg(ConstImageStampedPtr &_msg) {}

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompCamera)
}
