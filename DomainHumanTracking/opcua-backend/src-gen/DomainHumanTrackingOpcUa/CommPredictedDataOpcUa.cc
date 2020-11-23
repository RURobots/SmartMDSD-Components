#include "CommPredictedDataOpcUa.hh"

#define SERONET_NO_DEPRECATED
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ComplexType.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementPrimitives.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/SelfDescriptionArray.hpp>
#include <SeRoNetSDK/SeRoNet/CommunicationObjects/Description/ElementArray.hpp>


namespace SeRoNet {
namespace CommunicationObjects {
namespace Description {
	
// serialization for DomainHumanTrackingIDL::CommPredictedData
template <>
IDescription::shp_t SelfDescription(DomainHumanTrackingIDL::CommPredictedData *obj, std::string name)
{
	auto ret = std::make_shared<SeRoNet::CommunicationObjects::Description::ComplexType>(name);
	// add valid
	ret->add(
		SelfDescription(&(obj->valid), "Valid")
	);
	// add xMaxLimit
	ret->add(
		SelfDescription(&(obj->xMaxLimit), "XMaxLimit")
	);
	// add xMinLimit
	ret->add(
		SelfDescription(&(obj->xMinLimit), "XMinLimit")
	);
	// add yMaxLimit
	ret->add(
		SelfDescription(&(obj->yMaxLimit), "YMaxLimit")
	);
	// add yMinLimit
	ret->add(
		SelfDescription(&(obj->yMinLimit), "YMinLimit")
	);
	// add zMaxLimit
	ret->add(
		SelfDescription(&(obj->zMaxLimit), "ZMaxLimit")
	);
	// add zMinLimit
	ret->add(
		SelfDescription(&(obj->zMinLimit), "ZMinLimit")
	);
	return ret;
} // end SelfDescription

} // end namespace Description
} // end namespace CommunicationObjects
} // end namespace SeRoNet
