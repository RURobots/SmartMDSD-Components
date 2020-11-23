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
#ifndef DOMAINHUMANTRACKING_COMMPREDICTEDDATA_H_
#define DOMAINHUMANTRACKING_COMMPREDICTEDDATA_H_

#include "DomainHumanTracking/CommPredictedDataCore.hh"

namespace DomainHumanTracking {
		
class CommPredictedData : public CommPredictedDataCore {
	public:
		// default constructors
		CommPredictedData();
		
		/**
		 * Constructor to set all values.
		 * NOTE that you have to keep this constructor consistent with the model!
		 * Use  at your own choice.
		 *
		 * The preferred way to set values for initialization is:
		 *      CommRepository::MyCommObject obj;
		 *      obj.setX(1).setY(2).setZ(3)...;
		 */
		// CommPredictedData(const bool &valid = false, const double &xMaxLimit = 0.0, const double &xMinLimit = 0.0, const double &yMaxLimit = 0.0, const double &yMinLimit = 0.0, const double &zMaxLimit = 0.0, const double &zMinLimit = 0.0);
		
		CommPredictedData(const CommPredictedDataCore &commPredictedData);
		CommPredictedData(const DATATYPE &commPredictedData);
		virtual ~CommPredictedData();
		
		// use framework specific getter and setter methods from core (base) class
		using CommPredictedDataCore::get;
		using CommPredictedDataCore::set;
		
		//
		// feel free to add customized methods here
		//
};

inline std::ostream &operator<<(std::ostream &os, const CommPredictedData &co)
{
	co.to_ostream(os);
	return os;
}
	
} /* namespace DomainHumanTracking */
#endif /* DOMAINHUMANTRACKING_COMMPREDICTEDDATA_H_ */
