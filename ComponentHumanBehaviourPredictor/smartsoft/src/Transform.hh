
/******************************************************************************/
/*                                                                            */
/*  Name:           Transform.hh                                              */
/*  Purpose:        Implementation of a 3D transformation class               */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef Transform_hpp
#define Transform_hpp

#include <stdio.h>
#include <iostream>

#include "FourVector.hh"

class Transform {
    double product(int row, int column, const Transform& x) const;
public:
    Transform();
    double theMatrix[4][4]; // to hold the Transform
//    friend class DenavitHartenberg;
    //void apply(const fourvector& from, fourvector& to) const;
    void reset();
    void unit();
    void set(int row, int column, double value);
    void loadDHParms(float d, float theta, float a, float alpha);
    Transform operator *(const Transform& x) const ;
    fourvector operator *(const fourvector& x) const;
    Transform operator *(const double& x) const ;
    fourvector rotate(const fourvector& x) const; // just applies the rotation, not the translation
    void add(const Transform& x) ;
    void inverse(Transform& x) const ; // Make the transform the inverse of x
    void GenerateCrossProductMatrix(const fourvector& v);
    void GenerateRotationMatrix(const fourvector& unitAxis, double angle);
    fourvector get_x() const;
    fourvector get_y() const;
    fourvector get_z() const;
    fourvector get_offset() const;
    void setOffset(const fourvector offset);
    
    void printBases(std::ostream& os);
    
    void setRotateAboutX(const double theta);
    void setRotateAboutY(const double theta);
    void setRotateAboutZ(const double theta);

    friend std::ostream& operator<<(std::ostream& os, const Transform& T);
};

#endif /* Transform_hpp */
