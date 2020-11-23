/******************************************************************************/
/*                                                                            */
/*  Name:           Matrix.h                                                  */
/*  Purpose:        General purpose matrix maths class header                 */
/*  Author:         Mark Burgin                                               */
/*  Date:           14/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#ifndef Matrix_H_
#define Matrix_H_

#include "Vector.h"

namespace RUR_RobotMaths
{

class Matrix
{
    // Public Type Declaration
public:
    typedef enum
    {
        Rows,
        Columns
    } MatrixDimensions;

protected:
    typedef double* RowType;

    // Class Attributes

    // Class private members

protected:
    RowType* rows;
protected:
    int size[2];

    // Constructors and Destructors
public:
    Matrix(int i, int j);
public:
    Matrix(const Matrix& matrix);
public:
    virtual ~Matrix();
public:
    static Matrix Zeros(int i, int j);
public:
    static Matrix Ones(int i, int j);
public:
    static Matrix MinusOnes(int i, int j);
public:
    static Matrix Identity(int i, int j);

    // General Matrix Methods
public:
    virtual int Size(MatrixDimensions dimension) const;
public:
    virtual int RowCount() const;
public:
    virtual int ColumnCount() const;
public:
    virtual double GetValue(int i, int j) const;
public:
    virtual void SetValue(int i, int j, double value);
public:
    virtual Matrix ExtractSubMatrix(int i, int j) const;
public:
    virtual void SetRow(int i, Vector vector) const;
public:
    virtual void SetColumn(int j, Vector vector) const;
public:
    virtual void MakeZeros();
public:
    virtual void MakeOnes();
public:
    virtual void MakeMinusOnes();
public:
    virtual void MakeIdentity();

    // Matrix Mathematics
public:
    virtual Matrix Multiply(const Matrix& x) const;
public:
    virtual Vector Multiply(const Vector& x) const;
public:
    virtual Matrix Multiply(double x) const;
public:
    virtual Matrix Divide(double x) const;
public:
    virtual Matrix Plus(const Matrix& x) const;
public:
    virtual Matrix Minus(const Matrix& x) const;
public:
    virtual Matrix Transpose() const;
public:
    virtual double Determinant() const;
private:
    virtual Matrix InverseByLuDecomposition() const;
public:
    virtual Matrix Inverse() const;
public:
    virtual void LupDecompose(Matrix& l, Matrix& u, Matrix& p) const;
public:
    static Vector SolveLUxEqPB(const Matrix& l, const Matrix& u, const Matrix& p, const Vector& b);
private:
    static Vector SolveLxEqB(const Matrix& l, const Vector& b);
private:
    static Vector SolveUxEqB(const Matrix& u, const Vector& b);

    // Matrix Testing and Comparison
public:
    virtual bool IsSquare() const;
public:
    virtual bool IsSameSize(const Matrix& x) const;

    // Matrix I/O and Diagnostics
public:
    virtual const char* ToString() const;
public:
    static void FreeStringMemory(const char* stringPointer);
public:
    virtual void Print() const;

    // Matrix Operators
public:
   virtual Matrix operator *(const Matrix& y) const;
public:
   virtual Matrix operator *(double y) const;
public:
    friend Matrix operator *(double x, Matrix y);
public:
    virtual Vector operator *(const Vector& y) const;
public:
    virtual Matrix operator /(double y) const;
public:
    virtual Matrix operator +(const Matrix& y) const;
public:
    virtual Matrix operator -(const Matrix& y) const;
public:
    virtual RowType& operator[](const int i) const;
public:
    virtual Matrix& operator =(const Matrix& matrix);

    // Matrix Friends

    friend class MatrixTest; // Used only for unit testing
};

} // namespace RUR_RobotMaths

#endif /* #ifndef Matrix_H_ */
