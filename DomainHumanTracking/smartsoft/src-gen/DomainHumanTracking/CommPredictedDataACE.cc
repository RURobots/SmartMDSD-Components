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
#include "DomainHumanTracking/CommPredictedDataACE.hh"
#include <ace/SString.h>

// serialization operator for element CommPredictedData
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTrackingIDL::CommPredictedData &data)
{
	ACE_CDR::Boolean good_bit = true;
	// serialize list-element valid
	good_bit = good_bit && cdr.write_boolean(data.valid);
	// serialize list-element xMaxLimit
	good_bit = good_bit && cdr.write_double(data.xMaxLimit);
	// serialize list-element xMinLimit
	good_bit = good_bit && cdr.write_double(data.xMinLimit);
	// serialize list-element yMaxLimit
	good_bit = good_bit && cdr.write_double(data.yMaxLimit);
	// serialize list-element yMinLimit
	good_bit = good_bit && cdr.write_double(data.yMinLimit);
	// serialize list-element zMaxLimit
	good_bit = good_bit && cdr.write_double(data.zMaxLimit);
	// serialize list-element zMinLimit
	good_bit = good_bit && cdr.write_double(data.zMinLimit);
	
	return good_bit;
}

// de-serialization operator for element CommPredictedData
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTrackingIDL::CommPredictedData &data)
{
	ACE_CDR::Boolean good_bit = true;
	// deserialize type element valid
	good_bit = good_bit && cdr.read_boolean(data.valid);
	// deserialize type element xMaxLimit
	good_bit = good_bit && cdr.read_double(data.xMaxLimit);
	// deserialize type element xMinLimit
	good_bit = good_bit && cdr.read_double(data.xMinLimit);
	// deserialize type element yMaxLimit
	good_bit = good_bit && cdr.read_double(data.yMaxLimit);
	// deserialize type element yMinLimit
	good_bit = good_bit && cdr.read_double(data.yMinLimit);
	// deserialize type element zMaxLimit
	good_bit = good_bit && cdr.read_double(data.zMaxLimit);
	// deserialize type element zMinLimit
	good_bit = good_bit && cdr.read_double(data.zMinLimit);
	
	return good_bit;
}

// serialization operator for CommunicationObject CommPredictedData
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTracking::CommPredictedData &obj)
{
	return cdr << obj.get();
}

// de-serialization operator for CommunicationObject CommPredictedData
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTracking::CommPredictedData &obj)
{
	return cdr >> obj.set();
}