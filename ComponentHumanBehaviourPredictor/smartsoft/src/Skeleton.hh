
/******************************************************************************/
/*                                                                            */
/*  Name:           Skeleton.hh                                               */
/*  Purpose:        Implementation of a Skeleton kinematics class             */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/


#ifndef Skeleton_hpp
#define Skeleton_hpp

#include <stdio.h>
#include "Geometry.hh"
#include "Transform.hh"
#include "JointBaseClass.hh"
#include "JointPrismatic.hh"
#include "JointRotate.hh"
#include "Spine.hh"
#include "Arm.hh"

class Skeleton {
 
public:
    
     struct PositionVelocityDataSet {  // Skeleton Positions & Velocities supplied by vision system
        fourvector LeftHipPosition;
        fourvector LeftHipVelocity;
        fourvector RightHipPosition;
        fourvector RightHipVelocity;
        fourvector SpineBasePosition;
        fourvector SpineBaseVelocity;
        fourvector SpineTopPosition;
        fourvector SpineTopVelocity;
        fourvector ShoulderLeftPosition;
        fourvector ShoulderLeftVelocity;
        fourvector ShoulderRightPosition;
        fourvector ShoulderRightVelocity;
        fourvector ElbowLeftPosition;
        fourvector ElbowLeftVelocity;
        fourvector ElbowRightPosition;
        fourvector ElbowRightVelocity;
        fourvector WristLeftPosition;
        fourvector WristLeftVelocity;
        fourvector WristRightPosition;
        fourvector WristRightVelocity;
        fourvector HandLeftPosition;
        fourvector HandLeftVelocity;
        fourvector HandRightPosition;
        fourvector HandRightVelocity;
    };
    
    struct JointSet {  // Skeleton Positions & Velocities supplied by vision system
       fourvector SpineBase;
       fourvector SpineTopPosition;
       fourvector ShoulderLeftPosition;
       fourvector ShoulderRightPosition;
       fourvector ElbowLeftPosition;
       fourvector ElbowRightPosition;
       fourvector WristLeftPosition;
       fourvector WristRightPosition;
       fourvector HandLeftPosition;
       fourvector HandRightPosition;
   };

    struct JointSet_Scalars {
        double spineBaseLinear;
        double spineBaseFixedRotate;
        double spineBaseTopRotate;
        double spineTopLeftRotate;
        double spineTopRightRotate;
        double shoulderLeftZRotate;
        double shoulderRightZRotate;
        double shoulderLeftXRotate;
        double shoulderRightXRotate;
        double shoulderLeftUpperArmRotate;
        double shoulderRightUpperArmRotate;
        double elbowLeftRotate;
        double elbowRightRotate;
        double elbowLeftLowerArmRotate;
        double elbowRightLowerArmRotate;
        double wristLeftRotate;
        double wristRightRotate;
        double handLeftRotate;
        double handRightRotate;
    };
    
    struct linkLengths {
        double baseLength;
        double spineLength;
        double leftshoulderLength;
        double leftuperArmLength;
        double leftlowerArmLength;
        double lefthandlength;
        double rightshoulderLength;
        double rightuperArmLength;
        double rightlowerArmLength;
        double righthandlength;
    };
    
    struct PoseTransforms {
        Transform spineBaseLinear;
        Transform spineTopRotate;
        Transform LeftspineTopRotate;
        Transform LeftShoulderZRotate;
        Transform LeftShoulderXRotate;
        Transform LeftShoulderUpperArmRotate;
        Transform LeftElbowRotate;
        Transform LeftElbowLowerArmRotate;
        Transform LeftWristRotate;
        Transform LeftHandRotate;
        Transform RightspineTopRotate;
        Transform RightShoulderZRotate;
        Transform RightShoulderXRotate;
        Transform RightShoulderUpperArmRotate;
        Transform RightElbowRotate;
        Transform RightElbowLowerArmRotate;
        Transform RightWristRotate;
        Transform RightHandRotate;
    };
 
public:
    Spine theSpine;
    Arm theleftArm;
    Arm therightArm;

public:
    //SkeletonBackAndArm();
    
    void setPose(const PositionVelocityDataSet& visionData);
    
    void FindDHTransforms(JointSet jointPositions, JointSet jointAxes, fourvector SpineBaseVelocityUnitAxis) ;

    PoseTransforms ForwardKinematics(const double t) ;
    
    JointSet ExtractPositions() const;

    void findPredictedEnvelopeLimits( const double timePerStep, const int numSteps,
                                     /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax) ;
 

    
    friend std::ostream& operator<<(std::ostream& os, const Skeleton& x);
    friend std::ostream& operator<<(std::ostream& os, const Skeleton::PositionVelocityDataSet& x);
    friend std::ostream& operator<<(std::ostream& os, const Arm::PositionVelocityDataSet& x);
};
#endif /* Skeleton_hpp */
