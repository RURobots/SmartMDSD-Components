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
#ifndef DOMAINHUMANTRACKING_COMMHUMANPOSITIONPREDICTIONS_H_
#define DOMAINHUMANTRACKING_COMMHUMANPOSITIONPREDICTIONS_H_

#include "DomainHumanTracking/CommHumanPositionPredictionsCore.hh"

namespace DomainHumanTracking {
		
class CommHumanPositionPredictions : public CommHumanPositionPredictionsCore {
	public:
		// default constructors
		CommHumanPositionPredictions();
		
		/**
		 * Constructor to set all values.
		 * NOTE that you have to keep this constructor consistent with the model!
		 * Use  at your own choice.
		 *
		 * The preferred way to set values for initialization is:
		 *      CommRepository::MyCommObject obj;
		 *      obj.setX(1).setY(2).setZ(3)...;
		 */
		// CommHumanPositionPredictions(const std::vector<DomainHumanTracking::Comm3dVector> &vertices, const bool &valid = false);
		
		CommHumanPositionPredictions(const CommHumanPositionPredictionsCore &commHumanPositionPredictions);
		CommHumanPositionPredictions(const DATATYPE &commHumanPositionPredictions);
		virtual ~CommHumanPositionPredictions();
		
		// use framework specific getter and setter methods from core (base) class
		using CommHumanPositionPredictionsCore::get;
		using CommHumanPositionPredictionsCore::set;
		
		//
		// feel free to add customized methods here
		//
};

inline std::ostream &operator<<(std::ostream &os, const CommHumanPositionPredictions &co)
{
	co.to_ostream(os);
	return os;
}
	
} /* namespace DomainHumanTracking */
#endif /* DOMAINHUMANTRACKING_COMMHUMANPOSITIONPREDICTIONS_H_ */
