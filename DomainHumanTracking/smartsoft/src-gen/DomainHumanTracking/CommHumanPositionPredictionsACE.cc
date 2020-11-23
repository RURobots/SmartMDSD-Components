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
#include "DomainHumanTracking/CommHumanPositionPredictionsACE.hh"
#include <ace/SString.h>
#include "DomainHumanTracking/CommPredictedDataACE.hh"

// serialization operator for element CommHumanPositionPredictions
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTrackingIDL::CommHumanPositionPredictions &data)
{
	ACE_CDR::Boolean good_bit = true;
	// serialize list-element predictedData
	good_bit = good_bit && cdr << ACE_Utils::truncate_cast<ACE_CDR::ULong>(data.predictedData.size());
	std::vector<DomainHumanTrackingIDL::CommPredictedData>::const_iterator data_predictedDataIt;
	for(data_predictedDataIt=data.predictedData.begin(); data_predictedDataIt!=data.predictedData.end(); data_predictedDataIt++) {
		good_bit = good_bit && cdr << *data_predictedDataIt;
	}
	
	return good_bit;
}

// de-serialization operator for element CommHumanPositionPredictions
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTrackingIDL::CommHumanPositionPredictions &data)
{
	ACE_CDR::Boolean good_bit = true;
	// deserialize list-type element predictedData
	ACE_CDR::ULong data_predictedDataNbr;
	good_bit = good_bit && cdr >> data_predictedDataNbr;
	data.predictedData.clear();
	DomainHumanTrackingIDL::CommPredictedData data_predictedDataItem;
	for(ACE_CDR::ULong i=0; i<data_predictedDataNbr; ++i) {
		good_bit = good_bit && cdr >> data_predictedDataItem;
		data.predictedData.push_back(data_predictedDataItem);
	}
	
	return good_bit;
}

// serialization operator for CommunicationObject CommHumanPositionPredictions
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTracking::CommHumanPositionPredictions &obj)
{
	return cdr << obj.get();
}

// de-serialization operator for CommunicationObject CommHumanPositionPredictions
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTracking::CommHumanPositionPredictions &obj)
{
	return cdr >> obj.set();
}