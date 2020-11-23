
/******************************************************************************/
/*                                                                            */
/*  Name:           JointRotate.hh                                            */
/*  Purpose:        Implementation of a rotary joint                          */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef SpineBaseRotate_hpp
#define SpineBaseRotate_hpp

#include <stdio.h>

#include "JointBaseClass.hh"

class JointRotate : public JointBaseClass {

public:
    //JointRotate();
    
    virtual Transform ForwardKinematics(double velocity, const double t) const ;
    virtual Transform ForwardKinematicsAngle(double theta) const ;

};

#endif /* SpineBaseRotate_hpp */
