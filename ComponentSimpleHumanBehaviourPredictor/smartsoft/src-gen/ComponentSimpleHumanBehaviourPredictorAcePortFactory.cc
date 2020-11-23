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

#include "ComponentSimpleHumanBehaviourPredictorAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentSimpleHumanBehaviourPredictorAcePortFactory acePortFactory;

ComponentSimpleHumanBehaviourPredictorAcePortFactory::ComponentSimpleHumanBehaviourPredictorAcePortFactory()
{  
	componentImpl = 0;
	ComponentSimpleHumanBehaviourPredictor::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentSimpleHumanBehaviourPredictorAcePortFactory::~ComponentSimpleHumanBehaviourPredictorAcePortFactory()
{  }

void ComponentSimpleHumanBehaviourPredictorAcePortFactory::initialize(ComponentSimpleHumanBehaviourPredictor *component, int argc, char* argv[])
{
	if(component->connections.component.defaultScheduler != "DEFAULT") {
		ACE_Sched_Params sched_params(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(component->connections.component.defaultScheduler == "FIFO") {
			sched_params.policy(ACE_SCHED_FIFO);
			sched_params.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(component->connections.component.defaultScheduler == "RR") {
			sched_params.policy(ACE_SCHED_RR);
			sched_params.priority(ACE_THR_PRI_RR_MIN);
		}
		// create new instance of the SmartSoft component with customized scheuling parameters 
		componentImpl = new ComponentSimpleHumanBehaviourPredictorImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentSimpleHumanBehaviourPredictorImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentSimpleHumanBehaviourPredictorAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<DomainHumanTracking::CommHumanPositionsAndVelocities> * ComponentSimpleHumanBehaviourPredictorAcePortFactory::createHumanSkeletonsPushServiceIn()
{
	return new SmartACE::PushClient<DomainHumanTracking::CommHumanPositionsAndVelocities>(componentImpl);
}


Smart::IPushServerPattern<DomainHumanTracking::CommHumanPositionPredictions> * ComponentSimpleHumanBehaviourPredictorAcePortFactory::createHumanPredictionsPushServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainHumanTracking::CommHumanPositionPredictions>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions> * ComponentSimpleHumanBehaviourPredictorAcePortFactory::createHumanPredictionsRequestAnsw(const std::string &serviceName)
{
	return new SmartACE::QueryServer<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentSimpleHumanBehaviourPredictorAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentSimpleHumanBehaviourPredictorAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentSimpleHumanBehaviourPredictorAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
