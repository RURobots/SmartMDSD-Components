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
#include "DomainHumanTracking/Comm2dVectorCore.hh"

// serialization/deserialization operators
//#include "DomainHumanTracking/Comm2dVectorACE.hh"

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
	const char* Comm2dVectorCore::getCompiledHash()
	{
		return DomainHumanTrackingIDL::REPO_HASH;
	}
	
	void Comm2dVectorCore::getAllHashValues(std::list<std::string> &hashes)
	{
		// get own hash value
		hashes.push_back(getCompiledHash());
	}
	
	void Comm2dVectorCore::checkAllHashValues(std::list<std::string> &hashes)
	{
		// check own hash value
		if (strcmp(getCompiledHash(), hashes.front().c_str()) != 0)
		{
			std::cerr << "###################################################" << std::endl;
			std::cerr << "WARNING: HASHES OF COMMUNICATION OBJECTS MISSMATCH!" << std::endl;
			std::cerr << "Comm2dVectorCore hash" << std::endl;
			std::cerr << "Expected: " << getCompiledHash() << std::endl;
			std::cerr << "Received: " << hashes.front() << std::endl;
			std::cerr << "###################################################" << std::endl;
		}
		assert(strcmp(getCompiledHash(), hashes.front().c_str()) == 0);
		hashes.pop_front();
		
	}
	
	#ifdef ENABLE_HASH
	size_t Comm2dVectorCore::generateDataHash(const DATATYPE &data)
	{
		size_t seed = 0;
		
		boost::hash_combine(seed, data.x);
		boost::hash_combine(seed, data.y);
		
		return seed;
	}
	#endif
	
	// default constructor
	Comm2dVectorCore::Comm2dVectorCore()
	:	idl_Comm2dVector()
	{  
		setX(0.0);
		setY(0.0);
	}
	
	Comm2dVectorCore::Comm2dVectorCore(const DATATYPE &data)
	:	idl_Comm2dVector(data)
	{  }

	Comm2dVectorCore::~Comm2dVectorCore()
	{  }
	
	void Comm2dVectorCore::to_ostream(std::ostream &os) const
	{
	  os << "Comm2dVector(";
	  os << getX() << " ";
	  os << getY() << " ";
	  os << ") ";
	}
	
	// convert to xml stream
	void Comm2dVectorCore::to_xml(std::ostream &os, const std::string &indent) const {
		os << indent << "<x>" << getX() << "</x>";
		os << indent << "<y>" << getY() << "</y>";
	}
	
	// restore from xml stream
	void Comm2dVectorCore::from_xml(std::istream &is) {
		static const Smart::KnuthMorrisPratt kmp_x("<x>");
		static const Smart::KnuthMorrisPratt kmp_y("<y>");
		
		if(kmp_x.search(is)) {
			double xItem;
			is >> xItem;
			setX(xItem);
		}
		if(kmp_y.search(is)) {
			double yItem;
			is >> yItem;
			setY(yItem);
		}
	}
	
	/*
	void Comm2dVectorCore::get(ACE_Message_Block *&msg) const
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
		// (see Comm2dVectorACE.hh)
		cdr << idl_Comm2dVector;
		
	#ifdef ENABLE_HASH
		ACE_CDR::ULong data_hash = generateDataHash(idl_Comm2dVector);
		cdr << data_hash;
		// std::cout << "Comm2dVectorCore: current data hash: " << data_hash << std::endl;
	#endif
		
		// return a shallow copy of the serialized message 
		// (no data is actually copied, only the internal reference counter is incremented)
		// in order to prevent memory leaks the caller of this get(msg) method must
		// manually free the memory by calling the release() method of the message block msg
		msg = cdr.begin()->duplicate();
	}
	
	void Comm2dVectorCore::set(const ACE_Message_Block *msg)
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
		// (see Comm2dVectorACE.hh)
		cdr >> idl_Comm2dVector;
		
	#ifdef ENABLE_HASH
		ACE_CDR::Long data_hash;
		cdr >> data_hash;
		ACE_CDR::Long own_hash = generateDataHash(idl_Comm2dVector);
		assert(data_hash == own_hash);
		// std::cout << "Comm2dVectorCore: own data hash: " << own_hash << "; received data hash: " << data_hash << std::endl;
	#endif
	}
	*/
} /* namespace DomainHumanTracking */
