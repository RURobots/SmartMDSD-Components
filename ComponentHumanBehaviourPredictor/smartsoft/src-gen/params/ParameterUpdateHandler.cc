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
#include "ParameterUpdateHandler.hh"

#include "ComponentHumanBehaviourPredictor.hh"

SmartACE::CommParameterResponse ParamUpdateHandler::handleParameter(const SmartACE::CommParameterRequest& request)
{
	SmartACE::CommParameterResponse answer;

	std::string tag = request.getTag();
	std::cout<<"PARAMETER: "<<tag<<std::endl;
	
	if (tag == "COMMIT")
	{
		answer.setResponse(globalState.handleCOMMIT(commitState));
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			globalStateLock.acquire();
			// change the content of the globalState, however change only the generated content
			// without affecting potential user member variables (which is more intuitive for the user)
			globalState.setContent(commitState);
			globalStateLock.release();
		} else {
			// the commit validation check returned != OK
			// the commit state is rejected and is not copied into the global state
		}
	}
	else
	{
		/////////////////////////////////////////////////////////////////////
		// default new
		std::cout<<"ERROR wrong Parameter!"<<std::endl;
		answer.setResponse(SmartACE::ParamResponseType::INVALID);
	}
	

	std::cout<<"[handleQuery] PARAMETER "<<tag<<" DONE\n\n";

	return answer;
}


ParameterStateStruct ParamUpdateHandler::getGlobalState() const{
	SmartACE::SmartGuard g(globalStateLock);
	return this->globalState;
}


void ParamUpdateHandler::loadParameter(SmartACE::SmartIniParameter &parameter)
{
	/*
	 Parameters can be specified via command line -filename=<filename>

	 With this parameter present:
	 - The component will look for the file in the current working directory,
	 a path relative to the current directory or any absolute path
	 - The component will use the default values if the file cannot be found

	 With this parameter absent:
	 - <Name of Component>.ini will be read from current working directory, if found there
	 - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	 - Default values will be used if neither found in working directory or /etc
	 */

	// load parameters
	try
	{
		// print all known parameters
		parameter.print();

		//
		// load internal parameters (if any)
		//
		// parameter configuration
		if(parameter.getDouble("configuration", "decaySpeed", commitState.configuration.decaySpeed))
		{
			globalState.configuration.decaySpeed = commitState.configuration.decaySpeed;
		}
		// parameter sensorPosition
		if(parameter.getDouble("sensorPosition", "azimuth", commitState.sensorPosition.azimuth))
		{
			globalState.sensorPosition.azimuth = commitState.sensorPosition.azimuth;
		}
		if(parameter.getDouble("sensorPosition", "elevation", commitState.sensorPosition.elevation))
		{
			globalState.sensorPosition.elevation = commitState.sensorPosition.elevation;
		}
		if(parameter.getDouble("sensorPosition", "roll", commitState.sensorPosition.roll))
		{
			globalState.sensorPosition.roll = commitState.sensorPosition.roll;
		}
		if(parameter.getDouble("sensorPosition", "x", commitState.sensorPosition.x))
		{
			globalState.sensorPosition.x = commitState.sensorPosition.x;
		}
		if(parameter.getDouble("sensorPosition", "y", commitState.sensorPosition.y))
		{
			globalState.sensorPosition.y = commitState.sensorPosition.y;
		}
		if(parameter.getDouble("sensorPosition", "z", commitState.sensorPosition.z))
		{
			globalState.sensorPosition.z = commitState.sensorPosition.z;
		}
		// parameter settings
		if(parameter.getBoolean("settings", "debug_info", commitState.settings.debug_info))
		{
			globalState.settings.debug_info = commitState.settings.debug_info;
		}
		
		//
		// load extended parameters (if any)
		//
		
		//
		// load instance parameters (if a parameter definition was instantiated in the model)
		//

	} catch (const SmartACE::IniParameterError & e)
	{
		std::cerr << "Exception from parameter handling: " << e << std::endl;
	} catch (const std::exception &ex)
	{
		std::cerr << "Uncaught std:: exception" << ex.what() << std::endl;
	} catch (...)
	{
		std::cerr << "Uncaught exception" << std::endl;
	}
}
