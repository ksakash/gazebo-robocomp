
#ifndef COMMONHEAD_ICE
#define COMMONHEAD_ICE
/** \mainpage RoboComp Interfaces: HeadParams.ice
*
* \section intro_sec Introduction
* Interface for HeadParams  
* 
* PORT 10100 <br>   
*/

#include "JointMotor.ice"

/** \namespace RoboCompCommonHead
  *@brief Name space HeadParams
*/
module RoboCompCommonHead
{
	dictionary<string, RoboCompJointMotor::MotorParams> dmotorParams;
	struct THeadParams
	{
	  dmotorParams motorsParams;
	  string model;
	};

  dictionary <string, RoboCompJointMotor::MotorState> dmotorsState;
	struct THeadState   // In Radians
	{
	  dmotorsState motorsState;
	  bool isMoving;
	};


  /** \interface HeadParams
  * @brief interface HeadParams
  */  
  interface CommonHead
  {
	  /// Send cameras to ZEROPOS and set zero speed.
	  void resetHead();                   
	  /// Stop head where it is now
	  void stopHead();             
	  /// Set PanI servo to pan rads
	  void setPanLeft(float pan) throws RoboCompJointMotor::OutOfRangeException;         
	  /// Set PanD servo to pan rads
	  void setPanRight(float pan) throws RoboCompJointMotor::OutOfRangeException;  
	  /// Set Tilt servo to tilt rads
	  void setTilt(float tilt) throws RoboCompJointMotor::OutOfRangeException; 
	  /// Set neck servo to neck radians
	  void setNeck(float neck) throws RoboCompJointMotor::OutOfRangeException;
	  void saccadic2DLeft(float leftPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
	  void saccadic2DRight(float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
	  void saccadic3D(float leftPan, float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
	  void saccadic4D(float leftPan, float rightPan, float tilt, float neck) throws RoboCompJointMotor::OutOfRangeException; 
	  void setNMotorsPosition(RoboCompJointMotor::MotorGoalPositionList listGoals) throws RoboCompJointMotor::OutOfRangeException; 

	  RoboCompCommonHead::THeadParams getHeadParams();
	  void getHeadState(out RoboCompCommonHead::THeadState hState);
	  bool isMovingHead();
  };
};

#endif
