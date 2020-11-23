
/******************************************************************************/
/*                                                                            */
/*  Name:           JointBaseClass.cpp                                        */
/*  Purpose:        Base class for rotary/prismatic joints                    */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/


#include "JointBaseClass.hh"
#include "FourVector.hh"

using namespace std;

void JointBaseClass::Find_DHParams(Transform startPosTransform, fourvector EndPos, fourvector EndUnitAxis) {
    //cout << "startpos transform=" << startPosTransform << endl;
    fourvector commonNormal;
    fourvector commonAxisData;
    fourvector endAxisPoint;
    fourvector startAxisPoint;
    fourvector w;
    fourvector D;
    fourvector r0;
    fourvector lineToJointAxis;
    double a,d,theta,alpha, lineToJointAxisLen, finalOffsetAlongZAxistoEndPoint;
    Transform TZ;
    //cout << "========== Find_DHParams =========" << endl;
    //cout << "startPosTransform=" << endl << startPosTransform << endl;
    //cout << "EndPos=" << EndPos << endl;
    //cout << "EndUnitAxis=" << EndUnitAxis << endl;
    //cout << "==================================" << endl;
    fourvector startPos, startPos_xAxis, startPos_zAxis;
    startPos.init(startPosTransform.theMatrix[0][3],
                  startPosTransform.theMatrix[1][3],
                  startPosTransform.theMatrix[2][3],
                  1.0);
    
    startPos_xAxis.init(startPosTransform.theMatrix[0][0],
                        startPosTransform.theMatrix[1][0],
                        startPosTransform.theMatrix[2][0],
                        1.0);
    
    startPos_zAxis.init(startPosTransform.theMatrix[0][2],
                        startPosTransform.theMatrix[1][2],
                        startPosTransform.theMatrix[2][2],
                        1.0);
    cout << "P0=" << startPos << endl;
    cout << "u=" << startPos_zAxis << endl;
    cout << "Q0=" << EndPos << endl;
    cout << "v=" << startPos_xAxis << endl;
    G.FindCommonNormalEndPoints(/* inputs */  startPos, startPos_zAxis, EndPos, EndUnitAxis,
                              /* results */ startAxisPoint, endAxisPoint, w);
    D = startAxisPoint - startPos;
    d = D.norm();
    r0 = endAxisPoint - startAxisPoint;
    G.RequiredRotation(/* inputs */  startPos_xAxis, w, startPos_zAxis,
                     /* results */ theta);
    if ( G.dotProduct(D, startPos_zAxis) < 0)
        d =-d;
    a = r0.norm();
    if (G.dotProduct(r0, w) < 0)
        a = -a;
    G.RequiredRotation(/* inputs */  startPos_zAxis, EndUnitAxis, w,
                     /* results */ alpha);
    lineToJointAxis = EndPos - endAxisPoint;
    lineToJointAxisLen = lineToJointAxis.norm();
    finalOffsetAlongZAxistoEndPoint = lineToJointAxisLen;
    if (G.dotProduct(EndUnitAxis, lineToJointAxis) < 0)
        finalOffsetAlongZAxistoEndPoint = -lineToJointAxisLen;
    T.loadDHParms(d, theta, a, alpha);
    TZ.unit();
    TZ.set(2, 3, finalOffsetAlongZAxistoEndPoint);

    //cout << "T=" << T << " TZ=" << TZ << endl;
    toEndPointTransform = T * TZ;
    //cout << "local toEndPointTransform=" << toEndPointTransform << endl;
    //cout << "local endpoint transform offset=" << toEndPointTransform.get_offset() << endl;
    //cout << "Find_DHParams a=" << a << " alpha=" << alpha << " d=" << d << " theta=" << theta << " offset=" << finalOffsetAlongZAxistoEndPoint << endl;
    dhparams.a = a;
    dhparams.alpha = alpha;
    dhparams.d = d;
    dhparams.theta = theta;
};
