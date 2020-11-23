
/******************************************************************************/
/*                                                                            */
/*  Name:           Transform.cpp                                             */
/*  Purpose:        Implementation of a 3D transformation class               */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "Transform.hh"
#include <cmath>


Transform::Transform() {
    unit();
}

double Transform::product(int row, int column, const Transform& x) const {
    double total = 0.0;
    for (int i=0; i<4; i++)
    total += theMatrix[row][i] * x.theMatrix[i][column];
    return total;
}

Transform Transform::operator *(const Transform& x) const {
    Transform r;
    for (int row=0; row<4; row++)
       for (int column=0; column<4; column++)
            r.theMatrix[row][column] = product(row, column, x);
    return r;
};

fourvector Transform::operator *(const fourvector& x) const {
    fourvector r;
    for (int row=0; row<4; row++){
        r[row] = 0;
        for (int column=0; column<4; column++)
                r[row] += theMatrix[row][column] * x[column] ;
    }
    return r;
};

fourvector Transform::rotate(const fourvector& x) const {
    fourvector r;
    for (int row=0; row<3; row++){  // note only applying the rotation matrix
        r[row] = 0;
        for (int column=0; column<3; column++)
                r[row] += theMatrix[row][column] * x[column] ;
    }
    return r;
}

Transform Transform::operator *(const double& x) const {
    Transform r;
    for (int row=0; row<4; row++)
       for (int column=0; column<4; column++)
            r.theMatrix[row][column] = theMatrix[row][column] * x;
    return r;
}

void Transform::add(const Transform& x) {
    for (int row=0; row<4; row++) {
        for (int column=0; column<4; column++) {
            theMatrix[row][column] += x.theMatrix[row][column];
        }
    }
}

void Transform::reset() {
    for (int row=0; row<4; row++) {
        for (int column=0; column<4; column++) {
            theMatrix[row][column] = 0;
        }
    }
    theMatrix[3][3] = 1.0;
}

void Transform::unit() {
    reset();
    theMatrix[0][0] = 1.0;
    theMatrix[1][1] = 1.0;
    theMatrix[2][2] = 1.0;
    theMatrix[3][3] = 1.0;
}

void Transform::set(int row, int column, double value) {
    theMatrix[row][column] = value;
}

void Transform::setRotateAboutX(const double theta) {
    unit();
    theMatrix[1][1] = cos(theta);
    theMatrix[2][2] = cos(theta);
    theMatrix[1][2] = -sin(theta);
    theMatrix[2][1] = sin(theta);
}

void Transform::setRotateAboutY(const double theta) {
    unit();
    theMatrix[0][0] = cos(theta);
    theMatrix[2][2] = cos(theta);
    theMatrix[2][0] = -sin(theta);
    theMatrix[0][2] = sin(theta);
}

void Transform::setRotateAboutZ(const double theta) {
    unit();
    theMatrix[0][0] = cos(theta);
    theMatrix[0][1] = -sin(theta);
    theMatrix[1][1] = cos(theta);
    theMatrix[1][0] = sin(theta);
}


void Transform::loadDHParms(float d, float theta, float a, float alpha) {

    theMatrix[0][0] = cos(theta);
    theMatrix[0][1] = -sin(theta)*cos(alpha);
    theMatrix[0][2] = sin(theta)*sin(alpha);
    theMatrix[0][3] = a*cos(theta);
    theMatrix[1][0] = sin(theta);
    theMatrix[1][1] = cos(theta)*cos(alpha);
    theMatrix[1][2] = -cos(theta)*sin(alpha);
    theMatrix[1][3] = a*sin(theta);
    theMatrix[2][0] = 0;
    theMatrix[2][1] = sin(alpha);
    theMatrix[2][2] = cos(alpha);
    theMatrix[2][3] = d;
    theMatrix[3][0] = 0;
    theMatrix[3][1] = 0;
    theMatrix[3][2] = 0;
    theMatrix[3][3] = 1;
    //std::cout << "theMatrix=" << *this << std::endl;
}

void Transform::GenerateCrossProductMatrix(const fourvector& v) {
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
          theMatrix[i][j] = 0.0;
    /* row 0 */
    theMatrix[0][1] = -v[2];
    theMatrix[0][2] = v[1];
    /* row 1 */
    theMatrix[1][0] = v[2];
    theMatrix[1][2] = -v[0];
    /* row 2 */
    theMatrix[2][0] = -v[1];
    theMatrix[2][1] = v[0];
};

void Transform::GenerateRotationMatrix(const fourvector& unitAxis, double angle) {
    Transform E, E2, S, U, R;
    double sa = sin(angle);
    double ca = cos(angle);
    ca = 1.0 - ca;
    E.GenerateCrossProductMatrix(unitAxis);
    E2 = E * E;
    E2 = E2 * ca;
    S = E * sa;
    U.unit();
    reset();
    add(U);
    add(S);
    add(E2);
    theMatrix[3][3] = 1; // to compensate for the additions.
}
 
fourvector Transform::get_x() const {
    fourvector v;
    v[0] = theMatrix[0][0];
    v[1] = theMatrix[1][0];
    v[2] = theMatrix[2][0];
    v[3] = theMatrix[3][0];
    return v;
}

fourvector Transform::get_y() const {
    fourvector v;
    v[0] = theMatrix[0][1];
    v[1] = theMatrix[1][1];
    v[2] = theMatrix[2][1];
    v[3] = theMatrix[3][1];
    return v;
}

fourvector Transform::get_z() const {
    fourvector v;
    v[0] = theMatrix[0][2];
    v[1] = theMatrix[1][2];
    v[2] = theMatrix[2][2];
    v[3] = theMatrix[3][2];
    return v;
}

fourvector Transform::get_offset() const {
    fourvector v;
    v[0] = theMatrix[0][3];
    v[1] = theMatrix[1][3];
    v[2] = theMatrix[2][3];
    v[3] = theMatrix[3][3];
    return v;
}

void Transform::setOffset(const fourvector offset) {
    theMatrix[0][3] = offset[0];
    theMatrix[1][3] = offset[1];
    theMatrix[2][3] = offset[2];
    theMatrix[3][3] = 1.0;
}

void Transform::inverse(Transform& x) const { // Make x the inverse of this transform
    // first, make the rotation component = the transpose of the x rotation... R = x.Transpose(R)
    x.theMatrix[0][0] = theMatrix[0][0];
    x.theMatrix[1][1] = theMatrix[1][1];
    x.theMatrix[2][2] = theMatrix[2][2];
    x.theMatrix[0][1] = theMatrix[1][0];
    x.theMatrix[1][0] = theMatrix[0][1];
    x.theMatrix[0][2] = theMatrix[2][0];
    x.theMatrix[2][0] = theMatrix[0][2];
    x.theMatrix[1][2] = theMatrix[2][1];
    x.theMatrix[2][1] = theMatrix[1][2];
    x.theMatrix[3][0] = 0.0;
    x.theMatrix[3][1] = 0.0;
    x.theMatrix[3][2] = 0.0;
    // next the translation t = -R*x.t
    fourvector offset = get_offset();
    fourvector t = x.rotate(offset) * -1;
    //std::cout << "t=" << t << " x=" << x << std::endl;
    x.setOffset(t);
    //std::cout << "x=" << x << std::endl;
}

void Transform::printBases(std::ostream& os) {
    os << "x=" << get_x() << "y=" << get_y() << "z=" << get_z() << "offset=" << get_offset() << std::endl;
}

std::ostream& operator<<(std::ostream& os, const Transform& T) {
    std::ios::fmtflags old_settings = os.flags();
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::showpos;
    os.precision(3);
    os << std::endl;
    os << "[ " << std::setw(3) << T.theMatrix[0][0] << ", " << std::setw(3) << T.theMatrix[0][1] << ", " << std::setw(3) << T.theMatrix[0][2] << ", " << std::setw(3) << T.theMatrix[0][3] << " ]" << std::endl;
    os << "[ " << std::setw(3) << T.theMatrix[1][0] << ", " << std::setw(3) << T.theMatrix[1][1] << ", " << std::setw(3) << T.theMatrix[1][2] << ", " << std::setw(3) << T.theMatrix[1][3] << " ]" << std::endl;
    os << "[ " << std::setw(3) << T.theMatrix[2][0] << ", " << std::setw(3) << T.theMatrix[2][1] << ", " << std::setw(3) << T.theMatrix[2][2] << ", " << std::setw(3) << T.theMatrix[2][3] << " ]" << std::endl;
    os << "[ " << std::setw(3) << T.theMatrix[3][0] << ", " << std::setw(3) << T.theMatrix[3][1] << ", " << std::setw(3) << T.theMatrix[3][2] << ", " << std::setw(3) << T.theMatrix[3][3] << " ]" << std::endl;
    os.flags(old_settings);
    return os;
}
                                    
