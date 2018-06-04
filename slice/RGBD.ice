#ifndef ROBOCOMPRGBD_ICE
#define ROBOCOMPRGBD_ICE

#include "JointMotor.ice"

#include "GenericBase.ice"

module RoboCompRGBD{
	exception HardwareFailedException{string what;};
	enum Registration{None, DepthInColor, ColorInDepth};
	sequence <byte> imgType;
	sequence <float> depthType;

	struct ColorRGB{
		byte red;
		byte green; 
		byte blue;
	};
	struct PointXYZ
	{
		float x;
		float y;
		float z;
		float w;
	};

	sequence <float> DepthSeq;
	sequence <ColorRGB> ColorSeq;
	sequence <PointXYZ> PointSeq;

	struct CameraParameters
	{
		int focal;
		int width;
		int height;
		int size;
		int FPS;
	};
	struct TRGBDParams
	{
		CameraParameters color;
		CameraParameters depth;
		int timerPeriod;
		bool talkToBase;
		bool talkToJointMotor;
		string driver;
		string device;
	};

	interface RGBD
	{
		TRGBDParams getRGBDParams();
		idempotent
		void  setRegistration(Registration value)throws HardwareFailedException;
		idempotent Registration getRegistration()throws HardwareFailedException;
		idempotent
		void  getData(out imgType rgbMatrix, out depthType distanceMatrix, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		idempotent
		void  getDepthInIR(out depthType distanceMatrix, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		idempotent
		void  getImage(out ColorSeq color, out DepthSeq depth, out PointSeq points, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		void  getDepth(out DepthSeq depth, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		void  getRGB(out ColorSeq color, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		void  getXYZ(out PointSeq points, out RoboCompJointMotor::MotorStateMap hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
	};
};

#endif
