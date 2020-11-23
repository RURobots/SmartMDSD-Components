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

#ifndef COMPONENTSIMPLEHUMANBEHAVIOURPREDICTOR_ACE_PORTFACTORY_HH_
#define COMPONENTSIMPLEHUMANBEHAVIOURPREDICTOR_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentSimpleHumanBehaviourPredictorImpl.hh"

// include the main component-definition class
#include "ComponentSimpleHumanBehaviourPredictorPortFactoryInterface.hh"

class ComponentSimpleHumanBehaviourPredictorAcePortFactory: public ComponentSimpleHumanBehaviourPredictorPortFactoryInterface
{
private:
	ComponentSimpleHumanBehaviourPredictorImpl *componentImpl;
public:
	ComponentSimpleHumanBehaviourPredictorAcePortFactory();
	virtual ~ComponentSimpleHumanBehaviourPredictorAcePortFactory();

	virtual void initialize(ComponentSimpleHumanBehaviourPredictor *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<DomainHumanTracking::CommHumanPositionsAndVelocities> * createHumanSkeletonsPushServiceIn() override;
	
	virtual Smart::IPushServerPattern<DomainHumanTracking::CommHumanPositionPredictions> * createHumanPredictionsPushServiceOut(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions> * createHumanPredictionsRequestAnsw(const std::string &serviceName) override;
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTSIMPLEHUMANBEHAVIOURPREDICTOR_ACE_PORTFACTORY_HH_ */
