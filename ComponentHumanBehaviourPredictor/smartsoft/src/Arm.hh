
/******************************************************************************/
/*                                                                            */
/*  Name:           Arm.hh                                                    */
/*  Purpose:        Implementation of human arm kinematics                    */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef Arm_hpp
#define Arm_hpp

#include <stdio.h>

#include <stdio.h>
#include "Geometry.hh"
#include "Transform.hh"
#include "JointBaseClass.hh"
#include "JointPrismatic.hh"
#include "JointRotate.hh"

class Arm {
 
public:
    
    enum handedness{left, right};
    
    struct JointSet {
        fourvector Shoulder;
        fourvector Elbow;
        fourvector Wrist;
        fourvector Hand;
    };
    
    struct JointSet_Axes {
        fourvector spineTopRotate;
        fourvector ShoulderZRotate;
        fourvector ShoulderXRotate;
        fourvector ShoulderUpperArmRotate;
        fourvector ElbowRotate;
        fourvector ElbowLowerArmRotate;
        fourvector WristRotate;
        fourvector HandRotate;
    };

    struct JointSet_Scalars {
        double spineTopRotate;
        double shoulderZRotate;
        double shoulderXRotate;
        double shoulderUpperArmRotate;
        double elbowRotate;
        double elbowLowerArmRotate;
        double wristRotate;
        double handRotate;
    };
    
     
    struct PositionVelocityDataSet {  // Skeleton Positions & Velocities supplied by vision system
        fourvector ShoulderPosition;
        fourvector ShoulderVelocity;
        fourvector ElbowPosition;
        fourvector ElbowVelocity;
        fourvector WristPosition;
        fourvector WristVelocity;
        fourvector HandPosition;
        fourvector HandVelocity;
        //fourvector LeftHipPosition;
        //fourvector LeftHipVelocity;
        //fourvector RightHipPosition;
        //fourvector RightHipVelocity;
    };

    struct Pose {
        JointRotate spineTopRotate;  // from spine top to shoulder
        JointRotate ShoulderZRotate;
        JointRotate ShoulderXRotate;
        JointRotate ShoulderUpperArmRotate;
        JointRotate ElbowRotate;
        JointRotate ElbowLowerArmRotate;
        JointRotate WristRotate;
        JointRotate HandRotate;
    };

    struct PoseTransforms {
        Transform spineTopRotate;
        Transform ShoulderZRotate;
        Transform ShoulderXRotate;
        Transform ShoulderUpperArmRotate;
        Transform ElbowRotate;
        Transform ElbowLowerArmRotate;
        Transform WristRotate;
        Transform HandRotate;
    };

public:
    Pose P;
    Geometry G;
    PoseTransforms PTs;

private:
    //Transform cumulativeReferenceFrame;
    fourvector spineTopPosition;
    fourvector SpineTopVelocity;
    bool elbowBent;
    bool wristBent;
    JointSet_Scalars rotateRates;
    Transform spineTop;
    Transform SpineToShoulderRotate; // initaial rotation required to take shoulder to y-axis
    Arm::PositionVelocityDataSet localVisionData;

    Transform ProcessSpineTop(Transform initialReferenceFrame, const Transform& spineTopTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world, const fourvector SpineTopVelocity);
//    Transform ProcessShoulderRotate(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessShoulderZ(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessShoulderX(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessUpperArm(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessElbow(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessLowerArm(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessWrist(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);
    Transform ProcessHand(const Transform& PreElbowTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world);

public:
    //SkeletonBackAndArm();
    void setZeroPose(const handedness armSide, fourvector shoulderPosition, double upperArmLength, double lowerArmLength, double handLength);

    void setPose(const handedness armSide, const Arm::PositionVelocityDataSet& d, const Transform& spineTopTransform, const fourvector& SpineTopVelocity);
    
    void FindDHTransforms(JointSet jointPositions, JointSet jointAxes, fourvector SpineBaseVelocityUnitAxis) ;
 

    PoseTransforms ForwardKinematics(const double t) ;
    PoseTransforms ForwardKinematics(const JointSet_Scalars angles) ;

    PoseTransforms applyTransform(const Transform& T, const PoseTransforms& poses) const;
    JointSet applyTransform(const Transform& T, const JointSet& joints) const;
    
    JointSet ExtractPositions() const;

    void findPredictedEnvelopeLimits(const Transform& SpineTop, const double t,
                                     /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax) ;
 

    void ComputeSpineTopAxis(const fourvector positionShoulder1, const fourvector positionSpineTop,
                             const fourvector positionSpineBase, const fourvector velocityShoulder1, const fourvector velocitySpineTop,
                             /* output */ fourvector& spineTopRotUnitVector, double& angularRateSpineTopRotation) const;

    void ComputeShoulderAxis1(const fourvector positionElbow, const fourvector positionShoulder1, const fourvector velocityElbow,
                              const fourvector velocityShoulder1, const fourvector positionSpineTop, const fourvector velocitySpineTop,
                              /* outputs */ fourvector& Shoulder1UnitAxis, double& ShoulderOpeningRate) const;
    
    void ComputeShoulderAxis2(const fourvector positionWrist, const fourvector velocityWrist, const fourvector positionElbow,
                              const fourvector velocityElbow, const fourvector positionShoulder1, const fourvector positionSpineBase,
                              const fourvector  velocityVectorSpineBase, const fourvector SpineBaseUnitAxis,
                              double SpineBaseRotateRate,
                              /* output  */ fourvector& upperArmUnitAxis, double& wristRate) const;

    void ComputeElbowAxis(const fourvector positionElbow, const fourvector velocityElbow,
                          const fourvector positionShoulder1, const fourvector velocityShoulder1,
                          const fourvector positionWrist, const fourvector velocityWrist,
                          /* output */ fourvector& ElbowUnitAxis, double& ElbowRelativeRotation ) const;
    
    void ComputeWristAxis(const fourvector positionHand, const fourvector velocityHand,
                                            const fourvector positionWrist,
                                            const fourvector velocityWrist, const fourvector positionElbow,
                                            const fourvector velocityElbow,
                          /* output */ fourvector& WristUnitAxis, double& WristRelativeRotation ) const;
    void ComputeJointPositionAxisRate(const PositionVelocityDataSet visionData,
                                      /* output */ JointSet_Axes& returnResults_UnitAxes, JointSet_Scalars& returnResults_Rates) const;

    
    friend std::ostream& operator<<(std::ostream& os, const Arm::JointSet_Axes& x);
    friend std::ostream& operator<<(std::ostream& os, const Arm::JointSet_Scalars& x);
    friend std::ostream& operator<<(std::ostream& os, const Arm::PositionVelocityDataSet& x);
    friend std::ostream& operator<<(std::ostream& os, const Arm::JointSet& x) ;
};




#endif /* Arm_hpp */
