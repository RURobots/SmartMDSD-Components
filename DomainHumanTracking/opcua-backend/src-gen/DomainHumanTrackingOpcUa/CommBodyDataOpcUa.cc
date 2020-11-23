#include "CommBodyDataOpcUa.hh"

#define SERONET_NO_DEPRECATED
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ComplexType.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementPrimitives.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/SelfDescriptionArray.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementArray.hpp>

#include "CommBasicObjectsOpcUa/CommTimeStampOpcUa.hh"
#include "DomainHumanTrackingOpcUa/CommJointDataOpcUa.hh"

namespace SeRoNet {
namespace CommunicationObjects {
namespace Description {
	
// serialization for DomainHumanTrackingIDL::CommBodyData
template <>
IDescription::shp_t SelfDescription(DomainHumanTrackingIDL::CommBodyData *obj, std::string name)
{
	auto ret = std::make_shared<SeRoNet::CommunicationObjects::Description::ComplexType>(name);
	// add isTracked
	ret->add(
		SelfDescription(&(obj->isTracked), "IsTracked")
	);
	// add acquisitionTime
	ret->add(
		SelfDescription(&(obj->acquisitionTime), "AcquisitionTime")
	);
	// add leftHandState
	ret->add(
		SelfDescription(&(obj->leftHandState), "LeftHandState")
	);
	// add rightHandState
	ret->add(
		SelfDescription(&(obj->rightHandState), "RightHandState")
	);
	// add jointData
	ret->add(
		SelfDescription(&(obj->jointData), "JointData")
	);
	return ret;
} // end SelfDescription

} // end namespace Description
} // end namespace CommunicationObjects
} // end namespace SeRoNet
