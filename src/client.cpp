#include <Ice/Ice.h>
#include "LaserI.h"
#include <stdexcept>
 
using namespace std;
using namespace RoboCompLaser;

void printConfigData(TLaserConfigdata &laser_data)
{
    std::cerr << "Max Measures: " << laser_data.maxMeasures << std::endl;
    std::cerr << "Max Degrees: " << laser_data.maxDegrees << std::endl;
    std::cerr << "Max Range: " << laser_data.maxRange << std::endl;
    std::cerr << "Min Range: " << laser_data.minRange << std::endl;
    std::cerr << "Initial Range: " << laser_data.iniRange << std::endl;
    std::cerr << "End Range: " << laser_data.endRange << std::endl;
    std::cerr << "Angle Resolution: " << laser_data.angleRes << std::endl;
    std::cerr << "Initial Angle: " << laser_data.iniAngle << std::endl;
    std::cerr << "Device Diver: " << laser_data.deviceDriver << std::endl;
}
 
void printLaserScanData(TLaserData &laser_data)
{
    for(int i = 0; i < laser_data.size(); i++)
    {
        std::cerr << "Ray Index: " << i << " Ray Range: " << laser_data[i].dist << " Ray Angle: " << laser_data[i] << std::endl;
    }
}

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cerr << "Invalid Input!!!" << std::endl;
        return;
    }

    int toPublish = std::stoi(argv[1]);
    Ice::CommunicatorPtr ic;

    try
    {
        ic = Ice::initialize(argc, argv);
        Ice::ObjectPrx base = ich->stringToProxy("RoboCompLaser:default -p 10000");
        LaserPrx laser = LaserPrx::checkedCast(base);
        if(!laser)
        {
            throw std::runtime_error("Invalid proxy");
        }
 
        TLaserData scan_data = laser->getLaserData();
        LaserConfData  = laser->getLaserConfData();

        printConfiData(config_data);
        printLaserScanData(scan_data);

    }
    catch(const std::exception& e)
    {
        cerr << e.what() << endl;
        return 1;
    }
    return 0;
}