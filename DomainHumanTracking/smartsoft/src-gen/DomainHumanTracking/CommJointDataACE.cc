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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#include "DomainHumanTracking/CommJointDataACE.hh"
#include <ace/SString.h>
#include "DomainHumanTracking/Comm3dVectorACE.hh"
#include "DomainHumanTracking/Comm2dVectorACE.hh"
#include "DomainHumanTracking/enumJointTrackingStateTypeData.hh"

// serialization operator for element CommJointData
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTrackingIDL::CommJointData &data)
{
	ACE_CDR::Boolean good_bit = true;
	// serialize list-element isJointTracked
	good_bit = good_bit && cdr.write_long(data.isJointTracked);
	// serialize list-element jointPosition2D
	good_bit = good_bit && cdr << data.jointPosition2D;
	// serialize list-element jointVelocity2D
	good_bit = good_bit && cdr << data.jointVelocity2D;
	// serialize list-element jointPosition3d
	good_bit = good_bit && cdr << data.jointPosition3d;
	// serialize list-element jointVelocity3d
	good_bit = good_bit && cdr << data.jointVelocity3d;
	
	return good_bit;
}

// de-serialization operator for element CommJointData
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTrackingIDL::CommJointData &data)
{
	ACE_CDR::Boolean good_bit = true;
	// deserialize type element isJointTracked
	good_bit = good_bit && cdr.read_long(data.isJointTracked);
	// deserialize type element jointPosition2D
	good_bit = good_bit && cdr >> data.jointPosition2D;
	// deserialize type element jointVelocity2D
	good_bit = good_bit && cdr >> data.jointVelocity2D;
	// deserialize type element jointPosition3d
	good_bit = good_bit && cdr >> data.jointPosition3d;
	// deserialize type element jointVelocity3d
	good_bit = good_bit && cdr >> data.jointVelocity3d;
	
	return good_bit;
}

// serialization operator for CommunicationObject CommJointData
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTracking::CommJointData &obj)
{
	return cdr << obj.get();
}

// de-serialization operator for CommunicationObject CommJointData
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTracking::CommJointData &obj)
{
	return cdr >> obj.set();
}
