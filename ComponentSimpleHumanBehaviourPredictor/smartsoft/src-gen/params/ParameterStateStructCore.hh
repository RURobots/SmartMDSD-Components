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
#ifndef _PARAMETERSTATESTRUCTCORE_HH
#define _PARAMETERSTATESTRUCTCORE_HH

#include "aceSmartSoft.hh"

#include <iostream>

// forward declaration (in order to define validateCOMMIT(ParameterStateStruct) which is implemented in derived class)
class ParameterStateStruct;

class ParameterStateStructCore
{
	friend class ParamUpdateHandler;
public:
	
		///////////////////////////////////////////
		// Internal params
		///////////////////////////////////////////
		
		/**
		 * Definition of Parameter settings
		 */
		class settingsType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			bool debug_info;
		
		public:
			// default constructor
			settingsType() {
				debug_info = true;
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getDebug_info() const { return debug_info; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "settings(";
				os << "debug_info = " << debug_info << ", ";
				os << ")\n";
			}
			
		}; // end class settingsType
		
		/**
		 * Definition of Parameter sensorPosition
		 */
		class sensorPositionType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double azimuth;
			double elevation;
			double roll;
			double x;
			double y;
			double z;
		
		public:
			// default constructor
			sensorPositionType() {
				azimuth = 0.0;
				elevation = 0.0;
				roll = 0.0;
				x = 0.0;
				y = 0.0;
				z = 1.0;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getAzimuth() const { return azimuth; }
			inline double getElevation() const { return elevation; }
			inline double getRoll() const { return roll; }
			inline double getX() const { return x; }
			inline double getY() const { return y; }
			inline double getZ() const { return z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "sensorPosition(";
				os << "azimuth = " << azimuth << ", ";
				os << "elevation = " << elevation << ", ";
				os << "roll = " << roll << ", ";
				os << "x = " << x << ", ";
				os << "y = " << y << ", ";
				os << "z = " << z << ", ";
				os << ")\n";
			}
			
		}; // end class sensorPositionType
		
		/**
		 * Definition of Parameter configuration
		 */
		class configurationType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double decaySpeed;
		
		public:
			// default constructor
			configurationType() {
				decaySpeed = 0.3;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getDecaySpeed() const { return decaySpeed; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "configuration(";
				os << "decaySpeed = " << decaySpeed << ", ";
				os << ")\n";
			}
			
		}; // end class configurationType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
	
protected:

	// Internal params
	configurationType configuration;
	sensorPositionType sensorPosition;
	settingsType settings;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	configurationType getConfiguration() const {
		return configuration;
	}
	sensorPositionType getSensorPosition() const {
		return sensorPosition;
	}
	settingsType getSettings() const {
		return settings;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		configuration.to_ostream(os);
		sensorPosition.to_ostream(os);
		settings.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	}
};

#endif
