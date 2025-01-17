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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef DOMAINHUMANTRACKING_COMMJOINTDATA_H_
#define DOMAINHUMANTRACKING_COMMJOINTDATA_H_

#include "DomainHumanTracking/CommJointDataCore.hh"

namespace DomainHumanTracking {
		
class CommJointData : public CommJointDataCore {
	public:
		// default constructors
		CommJointData();
		
		/**
		 * Constructor to set all values.
		 * NOTE that you have to keep this constructor consistent with the model!
		 * Use  at your own choice.
		 *
		 * The preferred way to set values for initialization is:
		 *      CommRepository::MyCommObject obj;
		 *      obj.setX(1).setY(2).setZ(3)...;
		 */
		// CommJointData(const DomainHumanTracking::Comm2dVector &jointPosition2D, const DomainHumanTracking::Comm2dVector &jointVelocity2D, const DomainHumanTracking::Comm3dVector &jointPosition3d, const DomainHumanTracking::Comm3dVector &jointVelocity3d, const DomainHumanTracking::JointTrackingStateType &isJointTracked = DomainHumanTracking::JointTrackingStateType());
		
		CommJointData(const CommJointDataCore &commJointData);
		CommJointData(const DATATYPE &commJointData);
		virtual ~CommJointData();
		
		// use framework specific getter and setter methods from core (base) class
		using CommJointDataCore::get;
		using CommJointDataCore::set;
		
		//
		// feel free to add customized methods here
		//
};

inline std::ostream &operator<<(std::ostream &os, const CommJointData &co)
{
	co.to_ostream(os);
	return os;
}
	
} /* namespace DomainHumanTracking */
#endif /* DOMAINHUMANTRACKING_COMMJOINTDATA_H_ */
