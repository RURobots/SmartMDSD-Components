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
#include "DomainHumanTracking/CommHumanPositionPredictionsRequestCore.hh"

// serialization/deserialization operators
//#include "DomainHumanTracking/CommHumanPositionPredictionsRequestACE.hh"

// include the hash.idl containing the hash constant
#include "hash.hh"
#include <assert.h>
#include <cstring>
#include <iostream>

// SmartUtils used in from_xml method
#include "smartKnuthMorrisPratt.hh"

#ifdef ENABLE_HASH
	#include <boost/functional/hash.hpp>
#endif

namespace DomainHumanTracking 
{
	const char* CommHumanPositionPredictionsRequestCore::getCompiledHash()
	{
		return DomainHumanTrackingIDL::REPO_HASH;
	}
	
	void CommHumanPositionPredictionsRequestCore::getAllHashValues(std::list<std::string> &hashes)
	{
		// get own hash value
		hashes.push_back(getCompiledHash());
		// get hash value(s) for CommBasicObjects::CommTimeStamp(idl_CommHumanPositionPredictionsRequest.timeFrame)
		CommBasicObjects::CommTimeStamp::getAllHashValues(hashes);
		// get hash value(s) for CommBasicObjects::CommPose3d(idl_CommHumanPositionPredictionsRequest.frameOfReference)
		CommBasicObjects::CommPose3d::getAllHashValues(hashes);
	}
	
	void CommHumanPositionPredictionsRequestCore::checkAllHashValues(std::list<std::string> &hashes)
	{
		// check own hash value
		if (strcmp(getCompiledHash(), hashes.front().c_str()) != 0)
		{
			std::cerr << "###################################################" << std::endl;
			std::cerr << "WARNING: HASHES OF COMMUNICATION OBJECTS MISSMATCH!" << std::endl;
			std::cerr << "CommHumanPositionPredictionsRequestCore hash" << std::endl;
			std::cerr << "Expected: " << getCompiledHash() << std::endl;
			std::cerr << "Received: " << hashes.front() << std::endl;
			std::cerr << "###################################################" << std::endl;
		}
		assert(strcmp(getCompiledHash(), hashes.front().c_str()) == 0);
		hashes.pop_front();
		
		// check hash value(s) for CommBasicObjects::CommTimeStamp(idl_CommHumanPositionPredictionsRequest.timeFrame)
		CommBasicObjects::CommTimeStamp::checkAllHashValues(hashes);
		// check hash value(s) for CommBasicObjects::CommPose3d(idl_CommHumanPositionPredictionsRequest.frameOfReference)
		CommBasicObjects::CommPose3d::checkAllHashValues(hashes);
	}
	
	#ifdef ENABLE_HASH
	size_t CommHumanPositionPredictionsRequestCore::generateDataHash(const DATATYPE &data)
	{
		size_t seed = 0;
		
		seed += CommBasicObjects::CommTimeStamp::generateDataHash(data.timeFrame);
		seed += CommBasicObjects::CommPose3d::generateDataHash(data.frameOfReference);
		
		return seed;
	}
	#endif
	
	// default constructor
	CommHumanPositionPredictionsRequestCore::CommHumanPositionPredictionsRequestCore()
	:	idl_CommHumanPositionPredictionsRequest()
	{  
		setTimeFrame(CommBasicObjects::CommTimeStamp());
		setFrameOfReference(CommBasicObjects::CommPose3d());
	}
	
	CommHumanPositionPredictionsRequestCore::CommHumanPositionPredictionsRequestCore(const DATATYPE &data)
	:	idl_CommHumanPositionPredictionsRequest(data)
	{  }

	CommHumanPositionPredictionsRequestCore::~CommHumanPositionPredictionsRequestCore()
	{  }
	
	void CommHumanPositionPredictionsRequestCore::to_ostream(std::ostream &os) const
	{
	  os << "CommHumanPositionPredictionsRequest(";
	  getTimeFrame().to_ostream(os);
	  getFrameOfReference().to_ostream(os);
	  os << ") ";
	}
	
	// convert to xml stream
	void CommHumanPositionPredictionsRequestCore::to_xml(std::ostream &os, const std::string &indent) const {
		os << indent << "<timeFrame>";
		getTimeFrame().to_xml(os, indent);
		os << indent << "</timeFrame>";
		os << indent << "<frameOfReference>";
		getFrameOfReference().to_xml(os, indent);
		os << indent << "</frameOfReference>";
	}
	
	// restore from xml stream
	void CommHumanPositionPredictionsRequestCore::from_xml(std::istream &is) {
		static const Smart::KnuthMorrisPratt kmp_timeFrame("<timeFrame>");
		static const Smart::KnuthMorrisPratt kmp_frameOfReference("<frameOfReference>");
		
		if(kmp_timeFrame.search(is)) {
			CommBasicObjects::CommTimeStamp timeFrameItem;
			timeFrameItem.from_xml(is);
			setTimeFrame(timeFrameItem);
		}
		if(kmp_frameOfReference.search(is)) {
			CommBasicObjects::CommPose3d frameOfReferenceItem;
			frameOfReferenceItem.from_xml(is);
			setFrameOfReference(frameOfReferenceItem);
		}
	}
	
	/*
	void CommHumanPositionPredictionsRequestCore::get(ACE_Message_Block *&msg) const
	{
		// start with a default internal buffer size(will automatically grow if needed)
		ACE_OutputCDR cdr(ACE_DEFAULT_CDR_BUFSIZE);
		
		DomainHumanTrackingIDL::HashList hashes;
		getAllHashValues(hashes);
		cdr << static_cast<ACE_CDR::Long>(hashes.size());
		for(DomainHumanTrackingIDL::HashList::const_iterator it=hashes.begin(); it!=hashes.end(); it++)
		{
			cdr << ACE_CString(it->c_str());
		}
		
		// Here the actual serialization takes place using the OutputCDR serialization operator<<
		// (see CommHumanPositionPredictionsRequestACE.hh)
		cdr << idl_CommHumanPositionPredictionsRequest;
		
	#ifdef ENABLE_HASH
		ACE_CDR::ULong data_hash = generateDataHash(idl_CommHumanPositionPredictionsRequest);
		cdr << data_hash;
		// std::cout << "CommHumanPositionPredictionsRequestCore: current data hash: " << data_hash << std::endl;
	#endif
		
		// return a shallow copy of the serialized message 
		// (no data is actually copied, only the internal reference counter is incremented)
		// in order to prevent memory leaks the caller of this get(msg) method must
		// manually free the memory by calling the release() method of the message block msg
		msg = cdr.begin()->duplicate();
	}
	
	void CommHumanPositionPredictionsRequestCore::set(const ACE_Message_Block *msg)
	{
		ACE_InputCDR cdr(msg);
	
		DomainHumanTrackingIDL::HashList hashes;
		ACE_CDR::Long hashes_size;
		cdr >> hashes_size;
		for(int i=0; i<hashes_size; ++i) 
		{
			ACE_CString hash;
			cdr >> hash;
			hashes.push_back(hash.c_str());
		}
		checkAllHashValues(hashes);
		
		// Here the actual de-serialization takes place using the InputCDR serialization operator>>
		// (see CommHumanPositionPredictionsRequestACE.hh)
		cdr >> idl_CommHumanPositionPredictionsRequest;
		
	#ifdef ENABLE_HASH
		ACE_CDR::Long data_hash;
		cdr >> data_hash;
		ACE_CDR::Long own_hash = generateDataHash(idl_CommHumanPositionPredictionsRequest);
		assert(data_hash == own_hash);
		// std::cout << "CommHumanPositionPredictionsRequestCore: own data hash: " << own_hash << "; received data hash: " << data_hash << std::endl;
	#endif
	}
	*/
} /* namespace DomainHumanTracking */
