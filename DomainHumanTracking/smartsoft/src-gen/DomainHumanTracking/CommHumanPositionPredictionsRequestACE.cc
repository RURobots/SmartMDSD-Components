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
#include "DomainHumanTracking/CommHumanPositionPredictionsRequestACE.hh"
#include <ace/SString.h>
#include "CommBasicObjects/CommTimeStampACE.hh"
#include "CommBasicObjects/CommPose3dACE.hh"

// serialization operator for element CommHumanPositionPredictionsRequest
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTrackingIDL::CommHumanPositionPredictionsRequest &data)
{
	ACE_CDR::Boolean good_bit = true;
	// serialize list-element timeFrame
	good_bit = good_bit && cdr << data.timeFrame;
	// serialize list-element frameOfReference
	good_bit = good_bit && cdr << data.frameOfReference;
	
	return good_bit;
}

// de-serialization operator for element CommHumanPositionPredictionsRequest
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTrackingIDL::CommHumanPositionPredictionsRequest &data)
{
	ACE_CDR::Boolean good_bit = true;
	// deserialize type element timeFrame
	good_bit = good_bit && cdr >> data.timeFrame;
	// deserialize type element frameOfReference
	good_bit = good_bit && cdr >> data.frameOfReference;
	
	return good_bit;
}

// serialization operator for CommunicationObject CommHumanPositionPredictionsRequest
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTracking::CommHumanPositionPredictionsRequest &obj)
{
	return cdr << obj.get();
}

// de-serialization operator for CommunicationObject CommHumanPositionPredictionsRequest
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTracking::CommHumanPositionPredictionsRequest &obj)
{
	return cdr >> obj.set();
}