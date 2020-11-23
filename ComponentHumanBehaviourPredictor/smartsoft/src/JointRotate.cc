
/******************************************************************************/
/*                                                                            */
/*  Name:           JointRotate.cpp                                           */
/*  Purpose:        Implementation of a rotary joint                          */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/


#include "JointRotate.hh"
#include "Geometry.hh"

Transform JointRotate::ForwardKinematics(double velocity, const double t) const {
    Transform T;
    T = ForwardKinematicsAngle(velocity * t);
//    cout << T << endl;
    return T;
};

Transform JointRotate::ForwardKinematicsAngle(double theta) const {
    Transform T;
    DH_ParamsSet new_dhparams = dhparams;
//    cout << endl << endl << endl << new_dhparams << endl;

    new_dhparams.theta += theta;
    T.loadDHParms(new_dhparams.d, new_dhparams.theta, new_dhparams.a, new_dhparams.alpha);
//    cout << T << endl;
    return T;
}

/*
P.spineTopRotate.
T.loadDHParms(d, theta, a, alpha);
TZ.unit();
TZ.set(2, 3, finalOffsetAlongZAxistoEndPoint);
//TZ.set(2, 3, 10000);
cout << "T=" << T << " TZ=" << TZ << endl;
toEndPointTransform = T * TZ;
*/
