#include <Ice/Ice.h>
#include "Laser.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;
using namespace RoboCompLaser;

class LaserI : public Laser
{
public:
    LaserI(string _deviceName, string _topicName);
    ~LaserI();
    virtual TLaserData getLaserData(const::Ice::Current&) override;
    // virtual TLaserData getLaserAndBStateData(RoboCompGenericBase::TBaseState bState, const::Ice::Current&) override;
    virtual LaserConfData getLaserConfData(const::Ice::Current&) override;
private:
    void callback(ConstLaserScanStampedPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: std::string topic_name_;
    private: string device_name_;
    private: TLaserData LaserScanValues;
    private: LaserConfData LaserConfigData;
};