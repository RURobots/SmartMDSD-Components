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
#include "ComponentHumanSkeletonTracker.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentHumanSkeletonTrackerAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentHumanSkeletonTracker* ComponentHumanSkeletonTracker::_componentHumanSkeletonTracker = 0;

// constructor
ComponentHumanSkeletonTracker::ComponentHumanSkeletonTracker()
{
	std::cout << "constructor of ComponentHumanSkeletonTracker\n";
	
	// set all pointer members to NULL
	//componentHumanSkeletonTrackerParams = NULL;
	//coordinationPort = NULL;
	filterAndTrack = NULL;
	filterAndTrackTrigger = NULL;
	humanSkeletonsPushServiceOut = NULL;
	humanSkeletonsRequestAnsw = NULL;
	humanSkeletonsRequestAnswInputTaskTrigger = NULL;
	humanSkeletonsRequestAnswHandler = NULL;
	rGBDImagePushServiceIn = NULL;
	rGBDImagePushServiceInInputTaskTrigger = NULL;
	rGBDImagePushServiceInUpcallManager = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentHumanSkeletonTracker";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.humanSkeletonsPushServiceOut.serviceName = "HumanSkeletonsPushServiceOut";
	connections.humanSkeletonsPushServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.humanSkeletonsRequestAnsw.serviceName = "HumanSkeletonsRequestAnsw";
	connections.humanSkeletonsRequestAnsw.roboticMiddleware = "ACE_SmartSoft";
	connections.rGBDImagePushServiceIn.wiringName = "RGBDImagePushServiceIn";
	connections.rGBDImagePushServiceIn.serverName = "unknown";
	connections.rGBDImagePushServiceIn.serviceName = "unknown";
	connections.rGBDImagePushServiceIn.interval = 1;
	connections.rGBDImagePushServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.filterAndTrack.minActFreq = 0.0;
	connections.filterAndTrack.maxActFreq = 0.0;
	connections.filterAndTrack.trigger = "PeriodicTimer";
	connections.filterAndTrack.periodicActFreq = 1.0;
	// scheduling default parameters
	connections.filterAndTrack.scheduler = "DEFAULT";
	connections.filterAndTrack.priority = -1;
	connections.filterAndTrack.cpuAffinity = -1;
	
	// initialize members of ComponentHumanSkeletonTrackerROSExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentHumanSkeletonTrackerExtension
	
}

void ComponentHumanSkeletonTracker::addPortFactory(const std::string &name, ComponentHumanSkeletonTrackerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentHumanSkeletonTracker::addExtension(ComponentHumanSkeletonTrackerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentHumanSkeletonTracker::getComponentImpl()
{
	return dynamic_cast<ComponentHumanSkeletonTrackerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentHumanSkeletonTracker::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentHumanSkeletonTracker::connectRGBDImagePushServiceIn(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = rGBDImagePushServiceIn->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->rGBDImagePushServiceIn->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	rGBDImagePushServiceIn->subscribe(connections.rGBDImagePushServiceIn.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentHumanSkeletonTracker::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectRGBDImagePushServiceIn(connections.rGBDImagePushServiceIn.serverName, connections.rGBDImagePushServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentHumanSkeletonTracker::startAllTasks() {
	// start task FilterAndTrack
	if(connections.filterAndTrack.scheduler != "DEFAULT") {
		ACE_Sched_Params filterAndTrack_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.filterAndTrack.scheduler == "FIFO") {
			filterAndTrack_SchedParams.policy(ACE_SCHED_FIFO);
			filterAndTrack_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.filterAndTrack.scheduler == "RR") {
			filterAndTrack_SchedParams.policy(ACE_SCHED_RR);
			filterAndTrack_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		filterAndTrack->start(filterAndTrack_SchedParams, connections.filterAndTrack.cpuAffinity);
	} else {
		filterAndTrack->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentHumanSkeletonTracker::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentHumanSkeletonTracker::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "RGBDImagePushServiceIn") return rGBDImagePushServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentHumanSkeletonTracker::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentHumanSkeletonTrackerROSExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentHumanSkeletonTrackerExtension
		
		
		// initialize all registered port-factories
		for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
		{
			portFactory->second->initialize(this, argc, argv);
		}
		
		// initialize all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->initialize(this, argc, argv);
		}
		
		ComponentHumanSkeletonTrackerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentHumanSkeletonTrackerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentHumanSkeletonTracker is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		humanSkeletonsPushServiceOut = portFactoryRegistry[connections.humanSkeletonsPushServiceOut.roboticMiddleware]->createHumanSkeletonsPushServiceOut(connections.humanSkeletonsPushServiceOut.serviceName);
		humanSkeletonsRequestAnsw = portFactoryRegistry[connections.humanSkeletonsRequestAnsw.roboticMiddleware]->createHumanSkeletonsRequestAnsw(connections.humanSkeletonsRequestAnsw.serviceName);
		humanSkeletonsRequestAnswInputTaskTrigger = new Smart::QueryServerTaskTrigger<DomainHumanTracking::CommHumanPositionsAndVelocitiesRequest, DomainHumanTracking::CommHumanPositionsAndVelocities>(humanSkeletonsRequestAnsw);
		
		// create client ports
		rGBDImagePushServiceIn = portFactoryRegistry[connections.rGBDImagePushServiceIn.roboticMiddleware]->createRGBDImagePushServiceIn();
		
		// create InputTaskTriggers and UpcallManagers
		rGBDImagePushServiceInInputTaskTrigger = new Smart::InputTaskTrigger<DomainVision::CommRGBDImage>(rGBDImagePushServiceIn);
		rGBDImagePushServiceInUpcallManager = new RGBDImagePushServiceInUpcallManager(rGBDImagePushServiceIn);
		
		// create input-handler
		
		// create request-handlers
		humanSkeletonsRequestAnswHandler = new HumanSkeletonsRequestAnswHandler(humanSkeletonsRequestAnsw);
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.rGBDImagePushServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<DomainVision::CommRGBDImage>*>(rGBDImagePushServiceIn)->add(wiringSlave, connections.rGBDImagePushServiceIn.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task FilterAndTrack
		filterAndTrack = new FilterAndTrack(component);
		// configure input-links
		rGBDImagePushServiceInUpcallManager->attach(filterAndTrack);
		// configure task-trigger (if task is configurable)
		if(connections.filterAndTrack.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.filterAndTrack.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(filterAndTrack);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				filterAndTrackTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task FilterAndTrack" << std::endl;
			}
		} else if(connections.filterAndTrack.trigger == "DataTriggered") {
			filterAndTrackTrigger = getInputTaskTriggerFromString(connections.filterAndTrack.inPortRef);
			if(filterAndTrackTrigger != NULL) {
				filterAndTrackTrigger->attach(filterAndTrack, connections.filterAndTrack.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.filterAndTrack.inPortRef << " as activation source for Task FilterAndTrack" << std::endl;
			}
		} else
		{
			// setup default task-trigger as PeriodicTimer
			Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
			int microseconds = 1000*1000 / 1.0;
			if(microseconds > 0) {
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				triggerPtr->attach(filterAndTrack);
				// store trigger in class member
				filterAndTrackTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task FilterAndTrack" << std::endl;
			}
		}
		
		
		// link observers with subjects
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentHumanSkeletonTracker::run()
{
	stateSlave->acquire("init");
	// startup all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onStartup();
	}
	
	// startup all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onStartup();
	}
	stateSlave->release("init");
	
	// do not call this handler within the init state (see above) as this handler internally calls setStartupFinished() (this should be fixed in future)
	compHandler.onStartup();
	
	// this call blocks until the component is commanded to shutdown
	stateSlave->acquire("shutdown");
	
	// shutdown all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onShutdown();
	}
	
	// shutdown all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onShutdown();
	}
	
	if(connections.component.useLogger == true) {
		//FIXME: use logging
		//Smart::LOGGER->stopLogging();
	}
	
	compHandler.onShutdown();
	
	stateSlave->release("shutdown");
}

// clean-up component's resources
void ComponentHumanSkeletonTracker::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	rGBDImagePushServiceInUpcallManager->detach(filterAndTrack);
	// unlink the TaskTrigger
	if(filterAndTrackTrigger != NULL){
		filterAndTrackTrigger->detach(filterAndTrack);
		delete filterAndTrack;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete rGBDImagePushServiceInInputTaskTrigger;
	delete rGBDImagePushServiceInUpcallManager;

	// destroy client ports
	delete rGBDImagePushServiceIn;

	// destroy server ports
	delete humanSkeletonsPushServiceOut;
	delete humanSkeletonsRequestAnsw;
	delete humanSkeletonsRequestAnswInputTaskTrigger;
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	delete humanSkeletonsRequestAnswHandler;
	
	delete stateSlave;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	delete param;
	

	// destroy all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->destroy();
	}

	// destroy all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->destroy();
	}
	
	// destruction of ComponentHumanSkeletonTrackerROSExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentHumanSkeletonTrackerExtension
	
}

void ComponentHumanSkeletonTracker::loadParameter(int argc, char *argv[])
{
	/*
	 Parameters can be specified via command line --filename=<filename> or -f <filename>

	 With this parameter present:
	   - The component will look for the file in the current working directory,
	     a path relative to the current directory or any absolute path
	   - The component will use the default values if the file cannot be found

	 With this parameter absent:
	   - <Name of Component>.ini will be read from current working directory, if found there
	   - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	   - Default values will be used if neither found in working directory or /etc
	 */
	SmartACE::SmartIniParameter parameter;
	std::ifstream parameterfile;
	bool parameterFileFound = false;

	// load parameters
	try
	{
		// if paramfile is given as argument
		if(parameter.tryAddFileFromArgs(argc,argv,"filename", 'f'))
		{
			parameterFileFound = true;
			std::cout << "parameter file is loaded from an argv argument \n";
		} else if(parameter.searchFile("ComponentHumanSkeletonTracker.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentHumanSkeletonTracker.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentHumanSkeletonTracker.ini parameter file not found! (using default values or command line arguments)\n";
		}
		
		// add command line arguments to allow overwriting of parameters
		// from file
		parameter.addCommandLineArgs(argc,argv,"component");
		
		// initialize the naming service using the command line parameters parsed in the
		// SmartIniParameter class. The naming service parameters are expected to be in
		// the "component" parameter group.
		SmartACE::NAMING::instance()->checkForHelpArg(argc,argv);
		if(parameterFileFound) 
		{
			if(SmartACE::NAMING::instance()->init(parameter.getAllParametersFromGroup("component")) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		} else {
			if(SmartACE::NAMING::instance()->init(argc, argv) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		}
			
		// print all known parameters
		// parameter.print();
		
		//--- server port // client port // other parameter ---
		// load parameter
		parameter.getString("component", "name", connections.component.name);
		parameter.getString("component", "initialComponentMode", connections.component.initialComponentMode);
		if(parameter.checkIfParameterExists("component", "defaultScheduler")) {
			parameter.getString("component", "defaultScheduler", connections.component.defaultScheduler);
		}
		if(parameter.checkIfParameterExists("component", "useLogger")) {
			parameter.getBoolean("component", "useLogger", connections.component.useLogger);
		}
		
		// load parameters for client RGBDImagePushServiceIn
		parameter.getString("RGBDImagePushServiceIn", "serviceName", connections.rGBDImagePushServiceIn.serviceName);
		parameter.getString("RGBDImagePushServiceIn", "serverName", connections.rGBDImagePushServiceIn.serverName);
		parameter.getString("RGBDImagePushServiceIn", "wiringName", connections.rGBDImagePushServiceIn.wiringName);
		parameter.getInteger("RGBDImagePushServiceIn", "interval", connections.rGBDImagePushServiceIn.interval);
		if(parameter.checkIfParameterExists("RGBDImagePushServiceIn", "roboticMiddleware")) {
			parameter.getString("RGBDImagePushServiceIn", "roboticMiddleware", connections.rGBDImagePushServiceIn.roboticMiddleware);
		}
		
		// load parameters for server HumanSkeletonsPushServiceOut
		parameter.getString("HumanSkeletonsPushServiceOut", "serviceName", connections.humanSkeletonsPushServiceOut.serviceName);
		if(parameter.checkIfParameterExists("HumanSkeletonsPushServiceOut", "roboticMiddleware")) {
			parameter.getString("HumanSkeletonsPushServiceOut", "roboticMiddleware", connections.humanSkeletonsPushServiceOut.roboticMiddleware);
		}
		// load parameters for server HumanSkeletonsRequestAnsw
		parameter.getString("HumanSkeletonsRequestAnsw", "serviceName", connections.humanSkeletonsRequestAnsw.serviceName);
		if(parameter.checkIfParameterExists("HumanSkeletonsRequestAnsw", "roboticMiddleware")) {
			parameter.getString("HumanSkeletonsRequestAnsw", "roboticMiddleware", connections.humanSkeletonsRequestAnsw.roboticMiddleware);
		}
		
		// load parameters for task FilterAndTrack
		parameter.getDouble("FilterAndTrack", "minActFreqHz", connections.filterAndTrack.minActFreq);
		parameter.getDouble("FilterAndTrack", "maxActFreqHz", connections.filterAndTrack.maxActFreq);
		parameter.getString("FilterAndTrack", "triggerType", connections.filterAndTrack.trigger);
		if(connections.filterAndTrack.trigger == "PeriodicTimer") {
			parameter.getDouble("FilterAndTrack", "periodicActFreqHz", connections.filterAndTrack.periodicActFreq);
		} else if(connections.filterAndTrack.trigger == "DataTriggered") {
			parameter.getString("FilterAndTrack", "inPortRef", connections.filterAndTrack.inPortRef);
			parameter.getInteger("FilterAndTrack", "prescale", connections.filterAndTrack.prescale);
		}
		if(parameter.checkIfParameterExists("FilterAndTrack", "scheduler")) {
			parameter.getString("FilterAndTrack", "scheduler", connections.filterAndTrack.scheduler);
		}
		if(parameter.checkIfParameterExists("FilterAndTrack", "priority")) {
			parameter.getInteger("FilterAndTrack", "priority", connections.filterAndTrack.priority);
		}
		if(parameter.checkIfParameterExists("FilterAndTrack", "cpuAffinity")) {
			parameter.getInteger("FilterAndTrack", "cpuAffinity", connections.filterAndTrack.cpuAffinity);
		}
		
		// load parameters for ComponentHumanSkeletonTrackerROSExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentHumanSkeletonTrackerExtension
		
		
		// load parameters for all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->loadParameters(parameter);
		}
		
		paramHandler.loadParameter(parameter);
	
	} catch (const SmartACE::IniParameterError & e) {
		std::cerr << e.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std::exception: " << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}
