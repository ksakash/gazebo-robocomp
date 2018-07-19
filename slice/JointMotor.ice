#ifndef ROBOCOMPJOINTMOTOR_ICE
#define ROBOCOMPJOINTMOTOR_ICE

module RoboCompJointMotor{
	exception HardwareFailedException{string what;};
	exception OutOfRangeException{string what;};
	exception UnknownMotorException{string what;};
	exception CollisionException{string what;};
	["cpp:comparable"]
	struct MotorState{
		int p;
		int v;
		int temperature;
		bool isMoving;
		float pos; 
		float vel;
		float power;
		string timeStamp;
	};
	dictionary<string, MotorState>MotorStateMap;
	["cpp:comparable"]
	struct MotorParams{
		bool invertedSign;
		byte busId;
		float minPos;
		float maxPos;
		float maxVelocity;
		float zeroPos;
		float stepsRange;
		float maxDegrees;
		float offset;
		float unitsRange;
		string name;
	};
	sequence <MotorParams> MotorParamsList;
	["cpp:comparable"]
	struct BusParams{
		int numMotors;
		int baudRate;
		int basicPeriod;
		string handler;
		string device;
	};
	["cpp:comparable"]
	struct MotorGoalPosition{
		float position;
		float maxSpeed;
		string name;
	};
	sequence <MotorGoalPosition> MotorGoalPositionList;
	["cpp:comparable"]
	struct MotorGoalVelocity{
		float velocity;
		float maxAcc;
		string name;
	};
	sequence <MotorGoalVelocity> MotorGoalVelocityList;
	sequence <string> MotorList;

	interface JointMotor{
		void  setPosition(MotorGoalPosition goal)throws UnknownMotorException, HardwareFailedException, CollisionException;
		void  setVelocity(MotorGoalVelocity goal)throws UnknownMotorException, HardwareFailedException;
		void  setZeroPos(string name)throws UnknownMotorException, HardwareFailedException;
		void  setSyncPosition(MotorGoalPositionList listGoals)throws UnknownMotorException, HardwareFailedException;
		void  setSyncVelocity(MotorGoalVelocityList listGoals)throws UnknownMotorException, HardwareFailedException;
		void  setSyncZeroPos()throws UnknownMotorException, HardwareFailedException;
		MotorParams getMotorParams(string motor)throws UnknownMotorException;
		MotorState getMotorState(string motor)throws UnknownMotorException;
		MotorStateMap getMotorStateMap(MotorList mList)throws UnknownMotorException;
		MotorParamsList getAllMotorParams();
		BusParams getBusParams();
	};

};

#endif


