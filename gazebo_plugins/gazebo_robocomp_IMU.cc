#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include "gazebo_robocomp_IMU.hh"

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRoboCompIMU)
  ////////////////////////////////////////////////////

  GazeboRoboCompIMU::GazeboRoboCompIMU(): SensorPlugin()
  {
    accelerometer_data = ignition::math::Vector3d(0, 0, 0);
    gyroscope_data = ignition::math::Vector3d(0, 0, 0);
    orientation = ignition::math::Quaterniond(1,0,0,0);
    sensor_=NULL;
  }

  GazeboRoboCompIMU::~GazeboRoboCompIMU()
  {
    // gazebo_node_->shutdown();
  }

  void GazeboRoboCompIMU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    this->sdf_ = _sdf;
    this->world_name_ = _sensor->WorldName();

    this->sensor_ =  dynamic_cast<sensors::ImuSensor*>(_sensor.get());
    
    // this->sensor_ = _sensor;

    if (this->sensor_ == NULL)
    {
      gzthrow("ERROR: Sensor pointer is NULL");
      return;
    }

    this->sensor_->SetActive(true);

    // Topic to which the IMU data will be published
    if (this->sdf_->HasElement("topic"))
    {
      this->topic_name_ = sdf_->Get<std::string>("topic");
      std::cerr << "<topic> set to: "<< topic_name_ << std::endl;
    }
    else
    {
      this->topic_name_ = "/imu_data";
      std::cerr << "missing <topic>, set to /namespace/default: " << topic_name_ << std::endl;
    }

    // Update Rate
    if (this->sdf_->HasElement("update_rate"))
    {
      update_rate =  sdf_->Get<double>("update_rate");
      std::cerr << "<update_rate> set to: " << update_rate << std::endl;
    }
    else
    {
      update_rate = 1.0;
      std::cerr << "missing <update_rate>, set to default: " << update_rate << std::endl;
    }

    // Noise
    /*if (this->sdf_->HasElement("gaussianNoise"))
    {
      gaussian_noise =  sdf_->Get<double>("gaussianNoise");
      std::cerr << "<gaussianNoise> set to: " << gaussian_noise << std::endl;
    }
    else
    {
      gaussian_noise = 0.0;
      std::cerr << "missing <gaussianNoise>, set to default: " << gaussian_noise << std::endl;
    }*/

    // Create gazebo Node
    this->gazebo_node_ = transport::NodePtr(new  transport::Node());

    // Intialize the Node
    this->gazebo_node_->Init(this->world_name_);

    // Publisher to the topic
    imu_data_publisher_ = gazebo_node_->Advertise<gazebo::msgs::IMU>(topic_name_);

    connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRoboCompIMU::OnUpdate, this, _1));

    // this->imu_data_subscriber_ = gazebo_node_->Subscribe(this->sensor_->Topic(), &GazeboRoboCompIMU::OnUpdate, this);
    
    std::cerr << "this happened" << std::endl;

    // last_time = sensor->LastUpdateTime();
  }

  void GazeboRoboCompIMU::OnUpdate(const common::UpdateInfo &)
  {
    /*std::cerr << "inside update" << std::endl;
    orientation.X() = _msg->orientation().x();
    orientation.Y() = _msg->orientation().y();
    orientation.Z() = _msg->orientation().z();
    orientation.W() = _msg->orientation().w();

    accelerometer_data.X() = _msg->linear_acceleration().x();
    accelerometer_data.Y() = _msg->linear_acceleration().y();
    accelerometer_data.Z() = _msg->linear_acceleration().z();

    gyroscope_data.X() = _msg->angular_velocity().x();
    gyroscope_data.Y() = _msg->angular_velocity().y();
    gyroscope_data.Z() = _msg->angular_velocity().z();

    // publishing data
    // imu_data_publisher_->WaitForConnection();
    imu_data_publisher_->Publish(*(_msg));*/

    orientation = sensor_->Orientation();
    accelerometer_data = sensor_->LinearAcceleration();
    gyroscope_data = sensor_->AngularVelocity();

    std::cerr << "this also happened" << std::endl;

  }

  // double GazeboRoboCompIMU::GuassianKernel(double mu, double sigma)
  // {
  //   // generation of two normalized uniform random variables
  //   double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  //   double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  //
  //   // using Box-Muller transform to obtain a varaible with a standard normal distribution
  //   double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);
  //
  //   // scaling
  //   Z0 = sigma * Z0 + mu;
  //   return Z0;
  // }

}
