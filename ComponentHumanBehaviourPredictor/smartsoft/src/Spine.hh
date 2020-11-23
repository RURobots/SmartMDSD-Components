
/******************************************************************************/
/*                                                                            */
/*  Name:           Spine.hh                                                  */
/*  Purpose:        Implementation of Spine kinematics                        */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef Spine_hpp
#define Spine_hpp

#include <stdio.h>
#include "Geometry.hh"
#include "Transform.hh"
#include "JointBaseClass.hh"
#include "JointPrismatic.hh"
#include "JointRotate.hh"
#include "Arm.hh"

class Spine {
 
public:
    struct JointSet {
        fourvector SpineBase;
        fourvector SpineTop;
    };
    
    struct JointSet_Axes {
        fourvector spineBaseLinear;
        fourvector spineBaseFixedZRotate;  // fixed rotation of spine column to get it to the start position
        fourvector spineBaseFixedXRotate;  // fixed rotation of spine column to get it to the start position
        fourvector spineBaseFinalZRotate;  // fixed rotation of spine column to get it to the start position
        fourvector spineBaseTopRotate;  // axis used to move the spine top with the desired velocity vector.
    };

    struct JointSet_Scalars {
        double spineBaseLinear;
        double spineBaseFixedZRotate;
        double spineBaseFixedXRotate;
        double spineBaseFinalZRotate;
        double spineBaseTopRotate;
    };
    
    struct PositionVelocityDataSet {  // Skeleton Positions & Velocities supplied by vision system
        fourvector SpineBasePosition;
        fourvector SpineBaseVelocity;
        fourvector SpineTopPosition;
        fourvector SpineTopVelocity;
    };

public:
    Geometry G;
    double rotationRate;
    fourvector baseStartPosition;
    //fourvector baseEndPosition;
    fourvector basePositionCurrent;
    fourvector baseVelocity;
    fourvector spineRotationAxis;
    fourvector topRelative;
    fourvector spineTop;
 
    
public:

    
    void setPose(const PositionVelocityDataSet& visionData);
 
//    void FindDHTransforms(const Spine::PositionVelocityDataSet& visionData) ;
    Transform getSpineTopTransform() const;
    Transform getBaseTransform() const;
    Transform ForwardKinematics(const double t, fourvector& baseTravelDistance) ;
    
    JointSet ExtractPositions() const;

    // findPredictedEnvelopeLimits returns the transform for the top of spine as this is needed by the arm findPredictedEnvelopeLimits
    Transform findPredictedEnvelopeLimits(const double t,
                                     /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax) ;
 
    
    friend std::ostream& operator<<(std::ostream& os, const Spine::JointSet_Axes& x);
    friend std::ostream& operator<<(std::ostream& os, const Spine::JointSet_Scalars& x);
    friend std::ostream& operator<<(std::ostream& os, const Spine::PositionVelocityDataSet& x);
    friend std::ostream& operator<<(std::ostream& os, const Spine::JointSet& x) ;
};
#endif /* Spine_hpp */
