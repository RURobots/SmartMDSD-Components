
/******************************************************************************/
/*                                                                            */
/*  Name:           JointPrismatic.hh                                         */
/*  Purpose:        Implementation of a prismatic joint                       */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef SpineBasePrismatic_hpp
#define SpineBasePrismatic_hpp

#include <stdio.h>
#include "JointBaseClass.hh"

class JointPrismatic : public JointBaseClass {

    fourvector StartLocation;
    fourvector unit_velocity;
public:
    //JointPrismatic();
    
    virtual Transform ForwardKinematics(double velocity, const double t) const ;
    
    void loadLinear_OnlyTransform(fourvector StartLocation, fourvector unit_velocity);
    Transform ForwardKinematics_LinearOnly(double velocity, const double t) const ;
};

#endif /* SpineBasePrismatic_hpp */
