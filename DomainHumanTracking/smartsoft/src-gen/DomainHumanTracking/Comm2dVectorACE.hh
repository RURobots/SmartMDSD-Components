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
#ifndef DOMAINHUMANTRACKING_COMM2DVECTOR_ACE_H_
#define DOMAINHUMANTRACKING_COMM2DVECTOR_ACE_H_

#include "DomainHumanTracking/Comm2dVector.hh"

#include <ace/CDR_Stream.h>

// serialization operator for DataStructure Comm2dVector
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTrackingIDL::Comm2dVector &data);

// de-serialization operator for DataStructure Comm2dVector
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTrackingIDL::Comm2dVector &data);

// serialization operator for CommunicationObject Comm2dVector
ACE_CDR::Boolean operator<<(ACE_OutputCDR &cdr, const DomainHumanTracking::Comm2dVector &obj);

// de-serialization operator for CommunicationObject Comm2dVector
ACE_CDR::Boolean operator>>(ACE_InputCDR &cdr, DomainHumanTracking::Comm2dVector &obj);

#endif /* DOMAINHUMANTRACKING_COMM2DVECTOR_ACE_H_ */
