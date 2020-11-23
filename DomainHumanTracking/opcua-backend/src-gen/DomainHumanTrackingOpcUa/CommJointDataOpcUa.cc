#include "CommJointDataOpcUa.hh"

#define SERONET_NO_DEPRECATED
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ComplexType.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementPrimitives.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/SelfDescriptionArray.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementArray.hpp>

#include "DomainHumanTrackingOpcUa/Comm2dVectorOpcUa.hh"
#include "DomainHumanTrackingOpcUa/Comm2dVectorOpcUa.hh"
#include "DomainHumanTrackingOpcUa/Comm3dVectorOpcUa.hh"
#include "DomainHumanTrackingOpcUa/Comm3dVectorOpcUa.hh"

namespace SeRoNet {
namespace CommunicationObjects {
namespace Description {
	
// serialization for DomainHumanTrackingIDL::CommJointData
template <>
IDescription::shp_t SelfDescription(DomainHumanTrackingIDL::CommJointData *obj, std::string name)
{
	auto ret = std::make_shared<SeRoNet::CommunicationObjects::Description::ComplexType>(name);
	// add isJointTracked
	ret->add(
		SelfDescription(&(obj->isJointTracked), "IsJointTracked")
	);
	// add jointPosition2D
	ret->add(
		SelfDescription(&(obj->jointPosition2D), "JointPosition2D")
	);
	// add jointVelocity2D
	ret->add(
		SelfDescription(&(obj->jointVelocity2D), "JointVelocity2D")
	);
	// add jointPosition3d
	ret->add(
		SelfDescription(&(obj->jointPosition3d), "JointPosition3d")
	);
	// add jointVelocity3d
	ret->add(
		SelfDescription(&(obj->jointVelocity3d), "JointVelocity3d")
	);
	return ret;
} // end SelfDescription

} // end namespace Description
} // end namespace CommunicationObjects
} // end namespace SeRoNet
