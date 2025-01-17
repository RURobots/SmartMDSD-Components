//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------

#include "DomainHumanTracking/CommJointData.hh"

using namespace DomainHumanTracking;

CommJointData::CommJointData()
:	CommJointDataCore()
{  }


/**
 * Constructor to set all values.
 * NOTE that you have to keep this constructor consistent with the model!
 * Use  at your own choice.
 *
 * The preferred way to set values for initialization is:
 *      CommRepository::MyCommObject obj;
 *      obj.setX(1).setY(2).setZ(3)...;
CommJointData::CommJointData(const DomainHumanTracking::Comm2dVector &jointPosition2D, const DomainHumanTracking::Comm2dVector &jointVelocity2D, const DomainHumanTracking::Comm3dVector &jointPosition3d, const DomainHumanTracking::Comm3dVector &jointVelocity3d, const DomainHumanTracking::JointTrackingStateType &isJointTracked)
:	CommJointDataCore() // base constructor sets default values as defined in the model
{
	setIsJointTracked(isJointTracked);
	setJointPosition2D(jointPosition2D);
	setJointVelocity2D(jointVelocity2D);
	setJointPosition3d(jointPosition3d);
	setJointVelocity3d(jointVelocity3d);
}
 */

CommJointData::CommJointData(const CommJointDataCore &commJointData)
:	CommJointDataCore(commJointData)
{  }

CommJointData::CommJointData(const DATATYPE &commJointData)
:	CommJointDataCore(commJointData)
{  }

CommJointData::~CommJointData()
{  }
