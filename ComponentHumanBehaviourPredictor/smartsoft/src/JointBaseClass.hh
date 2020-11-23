
/******************************************************************************/
/*                                                                            */
/*  Name:           JointBaseClass.hh                                         */
/*  Purpose:        Base class for rotary/prismatic joints                    */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef JointBaseClass_hpp
#define JointBaseClass_hpp

#include <stdio.h>
#include <iostream>
#include "Transform.hh"
#include "DH_ParamSet.hh"
#include "Geometry.hh"


class JointBaseClass {
    
public:
    DH_ParamsSet dhparams;
    Transform T;
    Transform toEndPointTransform;
    double jointLowerLimit;  // the lower limit for angle/prismatic axis
    double jointUpperLimit;
    double DH_Param_zero_offset;
    Geometry G;

public:
    //JointBaseClass();
    
    void Find_DHParams(Transform startPosTransform, fourvector EndPos, fourvector EndUnitAxis) ;
    
    virtual Transform ForwardKinematics(double velocity, const double t) const = 0 ;
    
    friend std::ostream& operator<<(std::ostream& os, const JointBaseClass& x);
    
};

#endif /* JointBaseClass_hpp */
