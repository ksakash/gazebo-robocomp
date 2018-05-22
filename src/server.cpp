#include <Ice/Ice.h>
#include <LaserI.h>
 
using namespace std;
using namespace RoboCompLaser;
 
int main(int argc, char* argv[])
{
    Ice::CommunicatorPtr ic;
    try
    {
        ic = Ice::initialize(argc, argv);
        auto adapter = ic->createObjectAdapterWithEndpoints("RoboCompLaserAdapter", "default -p 10000");
        Ice::ObjectPtr object = new LaserI;
        adapter->add(object, Ice::stringToIdentity("RoboCompLaser"));
        adapter->activate();
        ic->waitForShutdown();
    }
    catch(const std::exception& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    if(ic) 
    {
        try 
        {
            ic->destroy();
        } 
        catch(const Ice::Exception& e) 
        {
            cerr << e << endl;
            return 1;
        }
    }
    return 0;
}