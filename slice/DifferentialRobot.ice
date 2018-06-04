#ifndef ROBOCOMPDIFFERENTIALROBOT_ICE
#define ROBOCOMPDIFFERENTIALROBOT_ICE

#include "GenericBase.ice"

module RoboCompDifferentialRobot{
    ["cpp:comparable"]
	struct TMechParams{
		int wheelRadius; 
		int axisLength;
		int encoderSteps;
		int gearRatio;
		float temp;
		float maxVelAdv;
		float maxVelRot;
		string device;
		string handler;
	};

	interface DifferentialRobot
	{
		void  getBaseState(out RoboCompGenericBase::TBaseState state)throws RoboCompGenericBase::HardwareFailedException;
		void  getBasePose(out int x, out int z, out float alpha)throws RoboCompGenericBase::HardwareFailedException;
		void  setSpeedBase(float adv, float rot)throws RoboCompGenericBase::HardwareFailedException;
		void  stopBase()throws RoboCompGenericBase::HardwareFailedException;
		void  resetOdometer()throws RoboCompGenericBase::HardwareFailedException;
		void  setOdometer(RoboCompGenericBase::TBaseState state)throws RoboCompGenericBase::HardwareFailedException;
		void  setOdometerPose(int x, int z, float alpha)throws RoboCompGenericBase::HardwareFailedException;
		void  correctOdometer(int x, int z, float alpha)throws RoboCompGenericBase::HardwareFailedException;
	};
};

#endif	