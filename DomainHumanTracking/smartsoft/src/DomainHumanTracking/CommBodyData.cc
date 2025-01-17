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

#include "DomainHumanTracking/CommBodyData.hh"

using namespace DomainHumanTracking;

CommBodyData::CommBodyData()
:	CommBodyDataCore()
{  }


/**
 * Constructor to set all values.
 * NOTE that you have to keep this constructor consistent with the model!
 * Use  at your own choice.
 *
 * The preferred way to set values for initialization is:
 *      CommRepository::MyCommObject obj;
 *      obj.setX(1).setY(2).setZ(3)...;
CommBodyData::CommBodyData(const CommBasicObjects::CommTimeStamp &acquisitionTime, const std::vector<DomainHumanTracking::CommJointData> &jointData, const bool &isTracked, const DomainHumanTracking::HandTrackingStateType &leftHandState, const DomainHumanTracking::HandTrackingStateType &rightHandState)
:	CommBodyDataCore() // base constructor sets default values as defined in the model
{
	setIsTracked(isTracked);
	setAcquisitionTime(acquisitionTime);
	setLeftHandState(leftHandState);
	setRightHandState(rightHandState);
	setJointData(jointData);
}
 */

CommBodyData::CommBodyData(const CommBodyDataCore &commBodyData)
:	CommBodyDataCore(commBodyData)
{  }

CommBodyData::CommBodyData(const DATATYPE &commBodyData)
:	CommBodyDataCore(commBodyData)
{  }

CommBodyData::~CommBodyData()
{  }
