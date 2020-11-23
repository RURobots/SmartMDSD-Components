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
#include "DomainHumanTracking/CommBodyDataCore.hh"

// serialization/deserialization operators
//#include "DomainHumanTracking/CommBodyDataACE.hh"

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
	const char* CommBodyDataCore::getCompiledHash()
	{
		return DomainHumanTrackingIDL::REPO_HASH;
	}
	
	void CommBodyDataCore::getAllHashValues(std::list<std::string> &hashes)
	{
		// get own hash value
		hashes.push_back(getCompiledHash());
		// get hash value(s) for CommBasicObjects::CommTimeStamp(idl_CommBodyData.acquisitionTime)
		CommBasicObjects::CommTimeStamp::getAllHashValues(hashes);
		// get hash value(s) for DomainHumanTracking::CommJointData(idl_CommBodyData.jointData)
		DomainHumanTracking::CommJointData::getAllHashValues(hashes);
	}
	
	void CommBodyDataCore::checkAllHashValues(std::list<std::string> &hashes)
	{
		// check own hash value
		if (strcmp(getCompiledHash(), hashes.front().c_str()) != 0)
		{
			std::cerr << "###################################################" << std::endl;
			std::cerr << "WARNING: HASHES OF COMMUNICATION OBJECTS MISSMATCH!" << std::endl;
			std::cerr << "CommBodyDataCore hash" << std::endl;
			std::cerr << "Expected: " << getCompiledHash() << std::endl;
			std::cerr << "Received: " << hashes.front() << std::endl;
			std::cerr << "###################################################" << std::endl;
		}
		assert(strcmp(getCompiledHash(), hashes.front().c_str()) == 0);
		hashes.pop_front();
		
		// check hash value(s) for CommBasicObjects::CommTimeStamp(idl_CommBodyData.acquisitionTime)
		CommBasicObjects::CommTimeStamp::checkAllHashValues(hashes);
		// check hash value(s) for DomainHumanTracking::CommJointData(idl_CommBodyData.jointData)
		DomainHumanTracking::CommJointData::checkAllHashValues(hashes);
	}
	
	#ifdef ENABLE_HASH
	size_t CommBodyDataCore::generateDataHash(const DATATYPE &data)
	{
		size_t seed = 0;
		
		boost::hash_combine(seed, data.isTracked);
		seed += CommBasicObjects::CommTimeStamp::generateDataHash(data.acquisitionTime);
		boost::hash_combine(seed, data.leftHandState);
		boost::hash_combine(seed, data.rightHandState);
		std::vector<DomainHumanTrackingIDL::CommJointData>::const_iterator data_jointDataIt;
		for(data_jointDataIt=data.jointData.begin(); data_jointDataIt!=data.jointData.end(); data_jointDataIt++) {
			seed += DomainHumanTracking::CommJointData::generateDataHash(*data_jointDataIt);
		}
		
		return seed;
	}
	#endif
	
	// default constructor
	CommBodyDataCore::CommBodyDataCore()
	:	idl_CommBodyData()
	{  
		setIsTracked(false);
		setAcquisitionTime(CommBasicObjects::CommTimeStamp());
		setLeftHandState(DomainHumanTracking::HandTrackingStateType());
		setRightHandState(DomainHumanTracking::HandTrackingStateType());
		setJointData(std::vector<DomainHumanTracking::CommJointData>());
	}
	
	CommBodyDataCore::CommBodyDataCore(const DATATYPE &data)
	:	idl_CommBodyData(data)
	{  }

	CommBodyDataCore::~CommBodyDataCore()
	{  }
	
	void CommBodyDataCore::to_ostream(std::ostream &os) const
	{
	  os << "CommBodyData(";
	  os << getIsTracked() << " ";
	  getAcquisitionTime().to_ostream(os);
	  getLeftHandState().to_ostream(os);
	  getRightHandState().to_ostream(os);
	  std::vector<DomainHumanTracking::CommJointData>::const_iterator jointDataIt;
	  std::vector<DomainHumanTracking::CommJointData> jointDataList = getJointDataCopy();
	  for(jointDataIt=jointDataList.begin(); jointDataIt!=jointDataList.end(); jointDataIt++) {
	  	jointDataIt->to_ostream(os);
	  }
	  os << ") ";
	}
	
	// convert to xml stream
	void CommBodyDataCore::to_xml(std::ostream &os, const std::string &indent) const {
		size_t counter = 0;
		
		os << indent << "<isTracked>" << getIsTracked() << "</isTracked>";
		os << indent << "<acquisitionTime>";
		getAcquisitionTime().to_xml(os, indent);
		os << indent << "</acquisitionTime>";
		os << indent << "<leftHandState>";
		getLeftHandState().to_xml(os, indent);
		os << indent << "</leftHandState>";
		os << indent << "<rightHandState>";
		getRightHandState().to_xml(os, indent);
		os << indent << "</rightHandState>";
		std::vector<DomainHumanTracking::CommJointData>::const_iterator jointDataIt;
		std::vector<DomainHumanTracking::CommJointData> jointDataList = getJointDataCopy();
		counter = 0;
		os << indent << "<jointDataList n=\"" << jointDataList.size() << "\">";
		for(jointDataIt=jointDataList.begin(); jointDataIt!=jointDataList.end(); jointDataIt++) {
			os << indent << "<jointData i=\"" << counter++ << "\">";
			jointDataIt->to_xml(os, indent);
			os << indent << "</jointData>";
		}
		os << indent << "</jointDataList>";
	}
	
	// restore from xml stream
	void CommBodyDataCore::from_xml(std::istream &is) {
		size_t counter = 0;
		
		static const Smart::KnuthMorrisPratt kmp_isTracked("<isTracked>");
		static const Smart::KnuthMorrisPratt kmp_acquisitionTime("<acquisitionTime>");
		static const Smart::KnuthMorrisPratt kmp_leftHandState("<leftHandState>");
		static const Smart::KnuthMorrisPratt kmp_rightHandState("<rightHandState>");
		static const Smart::KnuthMorrisPratt kmp_jointDataList("<jointDataList n=\"");
		static const Smart::KnuthMorrisPratt kmp_jointData("\">");
		
		if(kmp_isTracked.search(is)) {
			bool isTrackedItem;
			is >> isTrackedItem;
			setIsTracked(isTrackedItem);
		}
		if(kmp_acquisitionTime.search(is)) {
			CommBasicObjects::CommTimeStamp acquisitionTimeItem;
			acquisitionTimeItem.from_xml(is);
			setAcquisitionTime(acquisitionTimeItem);
		}
		if(kmp_leftHandState.search(is)) {
			DomainHumanTracking::HandTrackingStateType leftHandStateItem;
			leftHandStateItem.from_xml(is);
			setLeftHandState(leftHandStateItem);
		}
		if(kmp_rightHandState.search(is)) {
			DomainHumanTracking::HandTrackingStateType rightHandStateItem;
			rightHandStateItem.from_xml(is);
			setRightHandState(rightHandStateItem);
		}
		if(kmp_jointDataList.search(is)) {
			size_t numberElements;
			is >> numberElements;
			DomainHumanTracking::CommJointData jointDataItem;
			std::vector<DomainHumanTracking::CommJointData> jointDataList;
			kmp_jointData.search(is);
			for(counter=0; counter<numberElements; counter++) {
				if(kmp_jointData.search(is)) {
					jointDataItem.from_xml(is);
					jointDataList.push_back(jointDataItem);
				}
			}
			setJointData(jointDataList);
		}
	}
	
	/*
	void CommBodyDataCore::get(ACE_Message_Block *&msg) const
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
		// (see CommBodyDataACE.hh)
		cdr << idl_CommBodyData;
		
	#ifdef ENABLE_HASH
		ACE_CDR::ULong data_hash = generateDataHash(idl_CommBodyData);
		cdr << data_hash;
		// std::cout << "CommBodyDataCore: current data hash: " << data_hash << std::endl;
	#endif
		
		// return a shallow copy of the serialized message 
		// (no data is actually copied, only the internal reference counter is incremented)
		// in order to prevent memory leaks the caller of this get(msg) method must
		// manually free the memory by calling the release() method of the message block msg
		msg = cdr.begin()->duplicate();
	}
	
	void CommBodyDataCore::set(const ACE_Message_Block *msg)
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
		// (see CommBodyDataACE.hh)
		cdr >> idl_CommBodyData;
		
	#ifdef ENABLE_HASH
		ACE_CDR::Long data_hash;
		cdr >> data_hash;
		ACE_CDR::Long own_hash = generateDataHash(idl_CommBodyData);
		assert(data_hash == own_hash);
		// std::cout << "CommBodyDataCore: own data hash: " << own_hash << "; received data hash: " << data_hash << std::endl;
	#endif
	}
	*/
} /* namespace DomainHumanTracking */