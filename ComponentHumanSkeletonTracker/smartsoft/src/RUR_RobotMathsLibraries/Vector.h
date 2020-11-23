/******************************************************************************/
/*                                                                            */
/*  Name:           Vector.h                                                  */
/*  Purpose:        General purpose vector maths class header                 */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#ifndef Vector_h_
#define Vector_h_

namespace RUR_RobotMaths
{

class Vector
{
    // Class Members
protected:
    double *values;
protected:
    int count;

    // Class Constructors
public:
    explicit Vector(int i = 3);
public:
    Vector(double arrayValues[], int i);
public:
    Vector(const Vector& vector);
public:
    virtual ~Vector();
public:
    static Vector Zeros(int i);
public:
    static Vector Ones(int i);
public:
    static Vector MinusOnes(int i);

    // Class Methods
public:
    virtual int Size() const;
public:
    virtual double GetValue(int i) const;
public:
    virtual void SetValue(int i, double value);
public:
    virtual Vector Multiply(double x) const;
public:
    virtual Vector Divide(double x) const;
public:
    virtual Vector Plus(const Vector& x) const;
public:
    virtual Vector Minus(const Vector& x) const;
public:
    virtual double Dot(const Vector& x) const;
public:
    virtual Vector Cross(const Vector& x) const;
public:
    friend Vector Cross(const Vector& x, Vector y);
public:
    virtual double Norm(int n) const;
public:
    virtual void MakeZeros();
public:
    virtual void MakeOnes();
public:
    virtual void MakeMinusOnes();
public:
    virtual Vector& RecordMax(const Vector& vector, bool reset = false);
public:
    virtual Vector& RecordMin(const Vector& vector, bool reset = false);
public:
    virtual Vector PadToLength(int toLength);
public:
    virtual const char* ToString() const;
public:
    static void FreeStringMemory(const char* stringPointer);
public:
    virtual void Print() const;

    // Class Operators
public:
    virtual double &operator[](const int i) const;
public:
    virtual Vector operator *(double y) const;
public:
    friend Vector operator *(double x, Vector y);
public:
    virtual Vector operator /(double y) const;
public:
    virtual Vector operator +(const Vector& y) const;
public:
    virtual Vector operator -(const Vector& y) const;
public:
    virtual Vector& operator =(const Vector& vector);

    // Class friends
    friend class VectorTest; // Used only for unit testing
};

} // namespace RUR_Maths

#endif // #ifndef Vector_h_
