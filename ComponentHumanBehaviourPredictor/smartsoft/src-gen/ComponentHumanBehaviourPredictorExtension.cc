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

#include "ComponentHumanBehaviourPredictorExtension.hh"

ComponentHumanBehaviourPredictorExtension::ComponentHumanBehaviourPredictorExtension(const std::string &name)
:	extension_name(name)
{
	cancelled = false;
	COMP->addExtension(this);
}

ComponentHumanBehaviourPredictorExtension::~ComponentHumanBehaviourPredictorExtension()
{  }

int ComponentHumanBehaviourPredictorExtension::startExtensionThread()
{
	cancelled = false;
	// execute the task_execution() method in a new thread now
	extension_future = std::async(std::launch::async, &ComponentHumanBehaviourPredictorExtension::extensionExecution, this);
	return 0;
}

int ComponentHumanBehaviourPredictorExtension::stopExtensionThread(const std::chrono::steady_clock::duration &timeoutTime)
{
	cancelled = true;
	// wait on extension thread to exit
	if (extension_future.wait_for(timeoutTime) == std::future_status::timeout) {
		return -1;
    }
	return 0;
}

void ComponentHumanBehaviourPredictorExtension::loadParameters(const SmartACE::SmartIniParameter &parameter)
{
	// no-op
}

int ComponentHumanBehaviourPredictorExtension::onStartup()
{
	// default implementation just starts the internal thread
	return startExtensionThread();
}

int ComponentHumanBehaviourPredictorExtension::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	// default implementation stops the internal thread waiting up to timeoutTime until it is stopped (or a timeout occurs)
	return stopExtensionThread(timeoutTime);
}