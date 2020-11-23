#include "Comm3dVectorOpcUa.hh"

#define SERONET_NO_DEPRECATED
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ComplexType.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementPrimitives.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/SelfDescriptionArray.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementArray.hpp>


namespace SeRoNet {
namespace CommunicationObjects {
namespace Description {
	
// serialization for DomainHumanTrackingIDL::Comm3dVector
template <>
IDescription::shp_t SelfDescription(DomainHumanTrackingIDL::Comm3dVector *obj, std::string name)
{
	auto ret = std::make_shared<SeRoNet::CommunicationObjects::Description::ComplexType>(name);
	// add x
	ret->add(
		SelfDescription(&(obj->x), "X")
	);
	// add y
	ret->add(
		SelfDescription(&(obj->y), "Y")
	);
	// add z
	ret->add(
		SelfDescription(&(obj->z), "Z")
	);
	return ret;
} // end SelfDescription

} // end namespace Description
} // end namespace CommunicationObjects
} // end namespace SeRoNet
