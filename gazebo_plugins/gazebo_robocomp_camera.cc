#include "gazebo_robocomp_camera.hh"
#include <gazebo/rendering/Camera.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image.pb.h>

using namespace cv;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompCamera)

  GazeboRoboCompCamera::GazeboRoboCompCamera()
  {
    this->seed = 0;
    this->saveCount = 0;
    std::cerr << "Gazebo Camera Object created" << std::endl;
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
    this->sub_ = this->gazebo_node_->Subscribe("/gazebo/default/box/link/cam_sensor/image", &GazeboRoboCompCamera::OnMsg, this);
  }

  void GazeboRoboCompCamera::myMemCpy(void *dest, std::string &new_image, size_t n)
  {
    // Typecast src and dest addresses to (char *)
    // char *csrc = (char *)src;
    char *cdest = (char *)dest;
    std::string::iterator it=new_image.begin();
  
    // Copy contents of src[] to dest[]
    for (int i=0; i<n; i++) {
      cdest[i] = *it;
      ++it;
    }
  }

  void GazeboRoboCompCamera::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format)
  {
    if (seed == 0)
    {
      image_.create(_height, _width, CV_8UC3);
      std::cerr << "Image created!!!" << std::endl;

      // for (int i = 0; i < 10; i++)
      // {
      //   std::cerr << "Index: " << i << "Value is: " << _image[i] << std::endl;
      // }
      memcpy((unsigned char *) image_.data, &(_image[0]), _width*_height * 3);
      std::cerr << "Image Format: " << this->format << std::endl;

      // rendering::Camera::SaveFrame(_image, this->width, this->height, this->depth, this->format, "/tmp/camera/me.jpg");
    }
  
    // memcpy((unsigned char *) image_.data, &(_image[0]), _width*_height * 3);

    if (this->saveCount < 1)
    {        
      char tmp[1024];
      snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg", this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);

      this->parentSensor->GetCamera()->SaveFrame(_image, _width, _height, _depth, _format, tmp);
      std::cerr << "Saving frame [" << this->saveCount << "] as [" << tmp << "]\n";
      this->saveCount++;
    }

    if (seed == 0){
      gazebo::msgs::Image msg;
      msg.set_data(_image, _height*_width*_depth);

      std::string* new_image;
      new_image = msg.release_data();

      std::cerr << "Length: " << new_image->length() << std::endl;

      for ( std::string::iterator it=new_image->begin(); it!=new_image->end(); ++it)
        std::cout << *it;

      std::string::iterator it = new_image->begin();

      for (int i = 0; i < _height; i++)
      {
        for (int j = 0; j < _width; j++)
        {
          image_.at<cv::Vec3b>(j,i)[0]= *it;
          ++it;
          image_.at<cv::Vec3b>(j,i)[1]= *it;
          ++it;
          image_.at<cv::Vec3b>(j,i)[2]= *it;
          ++it;
        }
      }
      cv::imwrite( "Gazebo_Image.jpg", image_ );

      std::cerr << "Now Printing" << std::endl;
      for (int i = 0; i < _height; i++)
      {
        for (int j = 0; j < _width; j++)
        {
          std::cout << image_.at<cv::Vec3b>(j,i)[0];
          std::cout << image_.at<cv::Vec3b>(j,i)[1];
          std::cout << image_.at<cv::Vec3b>(j,i)[2];
        }
      }
    }
    
    cv::imshow("window", image_);
    std::cerr << "yoto" << std::endl;
    seed++;
  }

  void GazeboRoboCompCamera::OnMsg(ConstImageStampedPtr &_msg) {
    std::string new_image;
    // new_image = _msg->image().release_data();
    new_image = _msg->image().data();
    // for ( std::string::iterator it=new_image->begin(); it!=new_image->end(); ++it)
    //   std::cout << *it;

    // std::string::iterator it = new_image->begin();

    // for (int i = 0; i < _height; i++)
    // {
    //   for (int j = 0; j < _width; j++)
    //   {
    //     image_.at<cv::Vec3b>(j,i)[0]= *it;
    //     ++it;
    //     image_.at<cv::Vec3b>(j,i)[1]= *it;
    //     ++it;
    //     image_.at<cv::Vec3b>(j,i)[2]= *it;
    //     ++it;
    //   }
    // }
    this->image.create(_msg->image().height(), _msg->image().width(), CV_8UC3);
    std::cout << "height: " << _msg->image().height() << "width: " << _msg->image().width() << std::endl;
    if (image.data == NULL) std::cerr << "something is wrong" << std::endl;
    std::cerr << "hearing from this end" << std::endl;
    myMemCpy((unsigned char *)image.data, new_image, _msg->image().width()*_msg->image().height()*3);
    std::cerr << "this is not happening" << std::endl;
    if (this->seed == 1){
      cv::imwrite( "Gazebo_Image_.jpg", image );
    }
    seed++;
    
  }

}
