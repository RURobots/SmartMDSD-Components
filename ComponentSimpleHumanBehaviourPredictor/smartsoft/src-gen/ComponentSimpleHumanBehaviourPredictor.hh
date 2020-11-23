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
#ifndef _COMPONENTSIMPLEHUMANBEHAVIOURPREDICTOR_HH
#define _COMPONENTSIMPLEHUMANBEHAVIOURPREDICTOR_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentSimpleHumanBehaviourPredictorCore.hh"

#include "ComponentSimpleHumanBehaviourPredictorPortFactoryInterface.hh"
#include "ComponentSimpleHumanBehaviourPredictorExtension.hh"

// forward declarations
class ComponentSimpleHumanBehaviourPredictorPortFactoryInterface;
class ComponentSimpleHumanBehaviourPredictorExtension;

// includes for ComponentSimpleHumanBehaviourPredictorROSExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentSimpleHumanBehaviourPredictorExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <DomainHumanTracking/CommHumanPositionPredictions.hh>
#include <DomainHumanTracking/CommHumanPositionPredictionsACE.hh>
#include <DomainHumanTracking/CommHumanPositionPredictionsRequest.hh>
#include <DomainHumanTracking/CommHumanPositionPredictionsRequestACE.hh>
#include <DomainHumanTracking/CommHumanPositionsAndVelocities.hh>
#include <DomainHumanTracking/CommHumanPositionsAndVelocitiesACE.hh>

// include tasks
#include "SimpleHumanPredictionActivity.hh"
// include UpcallManagers
#include "HumanSkeletonsPushServiceInUpcallManager.hh"

// include input-handler(s)
// include request-handler(s)
#include "HumanPredictionsRequestAnswHandler.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentSimpleHumanBehaviourPredictor::instance()

class ComponentSimpleHumanBehaviourPredictor : public ComponentSimpleHumanBehaviourPredictorCore {
private:
	static ComponentSimpleHumanBehaviourPredictor *_componentSimpleHumanBehaviourPredictor;
	
	// constructor
	ComponentSimpleHumanBehaviourPredictor();
	
	// copy-constructor
	ComponentSimpleHumanBehaviourPredictor(const ComponentSimpleHumanBehaviourPredictor& cc);
	
	// destructor
	~ComponentSimpleHumanBehaviourPredictor() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentSimpleHumanBehaviourPredictorPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentSimpleHumanBehaviourPredictorExtension*> componentExtensionRegistry;
	
public:
	ParameterStateStruct getGlobalState() const
	{
		return paramHandler.getGlobalState();
	}
	
	ParameterStateStruct getParameters() const
	{
		return paramHandler.getGlobalState();
	}
	
	// define tasks
	Smart::TaskTriggerSubject* simpleHumanPredictionActivityTrigger;
	SimpleHumanPredictionActivity *simpleHumanPredictionActivity;
	
	// define input-ports
	// InputPort HumanSkeletonsPushServiceIn
	Smart::IPushClientPattern<DomainHumanTracking::CommHumanPositionsAndVelocities> *humanSkeletonsPushServiceIn;
	Smart::InputTaskTrigger<DomainHumanTracking::CommHumanPositionsAndVelocities> *humanSkeletonsPushServiceInInputTaskTrigger;
	HumanSkeletonsPushServiceInUpcallManager *humanSkeletonsPushServiceInUpcallManager;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::IPushServerPattern<DomainHumanTracking::CommHumanPositionPredictions> *humanPredictionsPushServiceOut;
	
	// define answer-ports
	Smart::IQueryServerPattern<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions> *humanPredictionsRequestAnsw;
	Smart::QueryServerTaskTrigger<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions> *humanPredictionsRequestAnswInputTaskTrigger;
	
	// define request-handlers
	HumanPredictionsRequestAnswHandler *humanPredictionsRequestAnswHandler;
	
	// definitions of ComponentSimpleHumanBehaviourPredictorROSExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentSimpleHumanBehaviourPredictorExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentSimpleHumanBehaviourPredictorPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentSimpleHumanBehaviourPredictorExtension *extension);
	
	/// this method allows to access the registered component-extensions (automatically converting to the actuall implementation type)
	template <typename T>
	T* getExtension(const std::string &name) {
		auto it = componentExtensionRegistry.find(name);
		if(it != componentExtensionRegistry.end()) {
			return dynamic_cast<T*>(it->second);
		}
		return 0;
	}
	
	/// initialize component's internal members
	void init(int argc, char *argv[]);
	
	/// execute the component's infrastructure
	void run();
	
	/// clean-up component's resources
	void fini();
	
	/// call this method to set the overall component into the Alive state (i.e. component is then ready to operate)
	void setStartupFinished();
	
	/// connect all component's client ports
	Smart::StatusCode connectAndStartAllServices();
	
	/// start all assocuated Activities
	void startAllTasks();
	
	/// start all associated timers
	void startAllTimers();
	
	Smart::StatusCode connectHumanSkeletonsPushServiceIn(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentSimpleHumanBehaviourPredictor* instance()
	{
		if(_componentSimpleHumanBehaviourPredictor == 0) {
			_componentSimpleHumanBehaviourPredictor = new ComponentSimpleHumanBehaviourPredictor();
		}
		return _componentSimpleHumanBehaviourPredictor;
	}
	
	static void deleteInstance() {
		if(_componentSimpleHumanBehaviourPredictor != 0) {
			delete _componentSimpleHumanBehaviourPredictor;
		}
	}
	
	// connections parameter
	struct connections_struct
	{
		// component struct
		struct component_struct
		{
			// the name of the component
			std::string name;
			std::string initialComponentMode;
			std::string defaultScheduler;
			bool useLogger;
		} component;
		
		//--- task parameter ---
		struct SimpleHumanPredictionActivity_struct {
			double minActFreq;
			double maxActFreq;
			std::string trigger;
			// only one of the following two params is 
			// actually used at run-time according 
			// to the system config model
			double periodicActFreq;
			// or
			std::string inPortRef;
			int prescale;
			// scheduling parameters
			std::string scheduler;
			int priority;
			int cpuAffinity;
		} simpleHumanPredictionActivity;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct HumanPredictionsPushServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} humanPredictionsPushServiceOut;
		struct HumanPredictionsRequestAnsw_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} humanPredictionsRequestAnsw;
	
		//--- client port parameter ---
		struct HumanSkeletonsPushServiceIn_struct {
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} humanSkeletonsPushServiceIn;
		
		// -- parameters for ComponentSimpleHumanBehaviourPredictorROSExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentSimpleHumanBehaviourPredictorExtension
		
	} connections;
};
#endif