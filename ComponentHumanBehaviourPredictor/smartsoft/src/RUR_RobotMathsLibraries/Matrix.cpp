/******************************************************************************/
/*                                                                            */
/*  Name:           Matrix.cpp                                                */
/*  Purpose:        General purpose matrix maths class implementation         */
/*					Matrices are stored in one of two different ways: either  */
/*					row optimised or column optimised which can make certain  */
/*					row and column operation more or less efficient.		  */
/*																			  */
/*					The storage for a row optimised matrix is as follows:	  */
/*																			  */
/*						Row[0] -> Col[0], Col[1] ... Col[m]					  */
/*						Row[1] -> Col[0], Col[1] ... Col[m]					  */
/*										:									  */
/*						Row[0] -> Col[0], Col[1] ... Col[m]					  */
/*																			  */
/*					The storage for a column optimised matrix is as follows:  */
/*																			  */
/*						Col[0] -> Row[0], Row[1] ... Row[m]					  */
/*						Col[1] -> Row[0], Row[1] ... Row[m]					  */
/*										:									  */
/*						Col[0] -> Row[0], Row[1] ... Row[m]					  */
/*																			  */
/*  Author:         Mark Burgin                                               */
/*  Date:           14/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

//#include <unistd.h>
#include <string.h> // Only used for the ToString method
#include <exception>
#include <cmath>
#include <limits>
#include "stdio.h" // Only used for Print method
using namespace std;
#include "MathsException.h"
#include "Matrix.h"
using namespace RUR_RobotMaths;

namespace RUR_RobotMaths
{

    // Constructors and Destructors

/* public */Matrix::Matrix(int i, int j)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of rows");
    }

    if (j <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of columns");
    }

    size[int(Rows)] = i;
    size[int(Columns)] = j;

    try
    {
        rows = new RowType[i];

        for (int row = 0; row < i; row ++)
        {
        	RowType newRow;
           	newRow = new double[j];
            rows[row] = newRow;
        }
    }
    catch (const exception& e)
    {
    	this->~Matrix(); // Explicitly call the destructor to delete the rows data structure
        throw MathsException("Error allocating memory for Matrix", e);
    }
}

/* public */Matrix::Matrix(const Matrix& matrix)
{
    size[Rows] = matrix.RowCount();
    size[Columns] = matrix.ColumnCount();

    try
    {
        rows = new RowType[RowCount()];

        for (int row = 0; row < RowCount(); row ++)
        {
        	RowType newRow;
          	newRow = new double[ColumnCount()];
            rows[row] = newRow;
        }
    }
    catch (const exception& e)
    {
    	this->~Matrix(); // Explicitly call the destructor to delete the "rows" data structure
        throw MathsException("Error allocating memory for Matrix", e);
    }

    for (int i = 0; i < matrix.RowCount(); i++)
    {
        for (int j = 0; j < matrix.ColumnCount(); j++)
        {
            this->rows[i][j] = matrix[i][j];
        }
    }
}

/* public, virtual */Matrix::~Matrix()
{
	for (int row = 0; row < size[Rows]; row++)
	{
		try
		{
			delete rows[row];
		}
		catch(...)
		{
			// Emtpy
		}
	}
	try
	{
		delete[] rows;
	}
	catch(...)
	{
		// Emtpy
	}
}

/* public, static */Matrix Matrix::Zeros(int i, int j)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of rows");
    }

    if (j <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of columns");
    }

    Matrix returnValue(i, j);

    for (int row = 0; row < i; row++)
    {
        for (int col = 0; col < j; col++)
        {
            returnValue.rows[row][col] = 0.0;
        }
    }

    return returnValue;
}

/* public, static */Matrix Matrix::Ones(int i, int j)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of rows");
    }

    if (j <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of columns");
    }

    Matrix returnValue(i, j);

    for (int row = 0; row < i; row++)
    {
        for (int col = 0; col < j; col++)
        {
            returnValue.rows[row][col] = 1.0;
        }
    }

    return returnValue;
}

/* public, static */Matrix Matrix::MinusOnes(int i, int j)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of rows");
    }

    if (j <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of columns");
    }

    Matrix returnValue(i, j);

    for (int row = 0; row < i; row++)
    {
        for (int col = 0; col < j; col++)
        {
            returnValue.rows[row][col] = -1.0;
        }
    }

    return returnValue;
}

/* public, static */Matrix Matrix::Identity(int i, int j)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of rows");
    }

    if (j <= 0)
    {
        throw MathsException("Cannot have a Matrix with 0 or negative number of columns");
    }

    Matrix returnValue(i, j);

    for (int row = 0; row < i; row++)
    {
        for (int col = 0; col < j; col++)
        {
            if (row == col)
            {
                returnValue.rows[row][col] = 1.0;
            }
            else
            {
                returnValue.rows[row][col] = 0.0;
            }
        }
    }

    return returnValue;
}

    // General Matrix Methods

/* public, virtual */int Matrix::Size(MatrixDimensions dimension) const
{
    return size[int(dimension)];
}

/* public, virtual */int Matrix::RowCount() const
{
    return Size(Rows);
}

/* public, virtual */int Matrix::ColumnCount() const
{
    return Size(Columns);
}

/* public, virtual */double Matrix::GetValue(int i, int j) const
{
    if ((i < 0) || (i >= size[Rows]) || ((j < 0) || (j >= size[Columns])))
    {
        throw MathsException("Index out of range getting matrix element");
    }

    return rows[i][j];
}

/* public, virtual */void Matrix::SetValue(int i, int j, double value)
{
    if ((i < 0) || (i >= size[Rows]) || ((j < 0) || (j >= size[Columns])))
    {
        throw MathsException("Index out of range setting matrix element");
    }

    rows[i][j] = value;
}

/* public, virtual */Matrix Matrix::ExtractSubMatrix(int i, int j) const
{
    Matrix returnValue (RowCount() - 1, ColumnCount() - 1);

    if (i >= RowCount() || (i < 0) || (j >= ColumnCount()) || (j < 0))
    {
        throw MathsException("Indexes for submatrix are outside bounds of array.");
    }

    int outputRow = 0;
    for (int row = 0; row < RowCount(); row++)
    {
        if (row != i)
        {
            int outputCol = 0;
            for (int col = 0; col < ColumnCount(); col++)
            {
                if (col != j)
                {
                    returnValue[outputRow][outputCol] = rows[row][col];
                    outputCol++;
                }
            }
            outputRow++;
        }
    }

    return returnValue;
}

/* public, virtual */void Matrix::SetRow(int i, Vector vector) const
{
    Vector returnValue(this->ColumnCount());

    if (i >= RowCount())
    {
        throw MathsException("Row not found in matrix");
    }

    if (ColumnCount() != vector.Size())
        {
            throw MathsException("Matrix and vector must have the same number of columns");
        }

    for (int col = 0; col < ColumnCount(); col++)
    {
        rows[i][col] = vector[col];
    }

}

/* public, virtual */void Matrix::SetColumn(int j, Vector vector) const
{

    if (j >= ColumnCount())
    {
        throw MathsException("Column not found in matrix");
    }

    if (RowCount() != vector.Size())
    {
    	throw MathsException("Matrix and vector must have the same number of rows");
    }

    for (int row = 0; row < RowCount(); row++)
    {
        rows[row][j] = vector[row];
    }

}

/* public, virtual */void Matrix::MakeZeros()
{
    for (int row = 0; row < RowCount(); row ++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            rows[row][col] = 0.0;
        }
    }
}

/* public, virtual */void Matrix::MakeOnes()
{
    for (int row = 0; row < RowCount(); row ++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            rows[row][col] = 1.0;
        }
    }
}

/* public, virtual */void Matrix::MakeMinusOnes()
{
    for (int row = 0; row < RowCount(); row ++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            rows[row][col] = -1.0;
        }
    }
}

/* public, virtual */void Matrix::MakeIdentity()
{
    for (int row = 0; row < RowCount(); row ++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            if (row == col)
            {
                rows[row][col] = 1.0;
            }
            else
            {
                rows[row][col] = 0.0;
            }
        }
    }
}


    // Matrix Mathematics

/* public, virtual */Matrix Matrix::Multiply(const Matrix& x) const
{
    Matrix returnValue(this->RowCount(), x.ColumnCount());

    if (this->ColumnCount() != x.RowCount())
    {
        throw MathsException("Matrices have incompatible sizes for multiplication");
    }

    for (int row = 0; row < this->RowCount(); row++)
    {
        for (int col = 0; col < x.ColumnCount(); col++)
        {
            returnValue[row][col] = 0.0;
            for (int k = 0; k < this->ColumnCount(); k++)
            {
                returnValue[row][col] += rows[row][k] * x[k][col];
            }
        }
    }

    return returnValue;
}

/* public, virtual */Vector Matrix::Multiply(const Vector& x) const
{
    Vector returnValue(this->RowCount());

    if (this->ColumnCount() != x.Size())
    {
        throw MathsException("Matrix and Column Vector have incompatible sizes for multiplication");
    }

    for (int row = 0; row < this->RowCount(); row++)
    {
        returnValue[row] = 0.0;
        for (int k =0; k < this->ColumnCount(); k++)
        {
            returnValue[row] += rows[row][k] * x[k]; //changed AP 26 08 2016
        }
    }

    return returnValue;
}

/* public, virtual */Matrix Matrix::Multiply(double x) const
{
    Matrix returnValue(RowCount(), ColumnCount());

    for (int row = 0; row < RowCount(); row++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            returnValue[row][col] = rows[row][col] * x;
        }
    }

    return returnValue;
}

/* public, virtual */Matrix Matrix::Divide(double x) const
{
    Matrix returnValue(RowCount(), ColumnCount());

    try
    {
        for (int row = 0; row < RowCount(); row++)
        {
            for (int col = 0; col < ColumnCount(); col++)
            {
                returnValue[row][col] = rows[row][col] / x;
            }
        }
    }
    catch (const exception& e)
    {
        throw MathsException("Divided matrix by 0 scaler", e);
    }

    return returnValue;
}

/* public, virtual */Matrix Matrix::Plus(const Matrix& x) const
{
    Matrix returnValue(this->RowCount(), this->ColumnCount());

    if (!this->IsSameSize(x))
    {
        throw MathsException("Matrices have incompatible sizes for addition");
    }

    for (int row = 0; row < RowCount(); row++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            returnValue[row][col] = rows[row][col] + x[row][col];
        }
    }

    return returnValue;
}

/* public, virtual */Matrix Matrix::Minus(const Matrix& x) const
{
    Matrix returnValue(this->RowCount(), this->ColumnCount());

    if (!this->IsSameSize(x))
    {
        throw MathsException("Matrices ahave incompatible sizes for subtraction");
    }

    for (int row = 0; row < RowCount(); row++)
    {
        for (int col = 0; col < ColumnCount(); col++)
        {
            returnValue[row][col] = rows[row][col] - x[row][col];
        }
    }

    return returnValue;
}

/* public, virtual */Matrix Matrix::Transpose() const
{
    Matrix returnValue(this->ColumnCount(), this->RowCount());

    for (int row = 0; row < this->RowCount(); row++)
    {
        for (int col = 0; col < this->ColumnCount(); col++)
        {
            returnValue[col][row] = rows[row][col];
        }
    }

    return returnValue;
}

/* public, virtual */double Matrix::Determinant() const
{
    double returnValue = 0;

    if (!IsSquare())
    {
        throw MathsException("Matrix is not square.  Determinant is only defined on square matrices.");
    }

    if (this->RowCount() == 2)
    {
        returnValue = (rows[0][0] * rows[1][1]) - (rows[0][1] * rows[1][0]);
    }
    else
    {
        int sign = 1;
        for (int col = 0; col < ColumnCount(); col++)
        {
            Matrix subMatrix(this->RowCount() - 1, ColumnCount() - 1);
            subMatrix = ExtractSubMatrix(0, col);

            returnValue += sign * rows[0][col] * subMatrix.Determinant();

            sign = -1 * sign;
        }
    }

    return returnValue;
}

/* private, virtual */Matrix Matrix::InverseByLuDecomposition() const
{
    if (!IsSquare())
    {
        throw MathsException("Matrix not square.  Only square matrices are invertible using LU Decomposition.  Perhaps use PseudoInverse instead");
    }

    Matrix returnValue = Matrix(RowCount(), ColumnCount());
    Vector b = Vector::Zeros(RowCount());
    Matrix l = Matrix(RowCount(), ColumnCount());
    Matrix u = Matrix(RowCount(), ColumnCount());
    Matrix p = Matrix(RowCount(), ColumnCount());
    Vector x = Vector(RowCount());

    try
    {
    	LupDecompose(l, u, p);
		for (int column = 0; column < ColumnCount(); column++)
		{
			b[column] = 1.0;
			x = SolveLUxEqPB(l, u, p, b);
			b[column] = 0.0;
			returnValue.SetColumn(column, x);
		}
	}
	catch (const exception& e)
	{
		throw MathsException("Matrix cannot be inverted using LuDecomposition.", e);
	}

	return returnValue;
}

/* public, virtual */Matrix Matrix::Inverse() const
{
    Matrix returnValue(this->RowCount(), this->ColumnCount());

    if (!this->IsSquare())
    {
        throw MathsException("Matrix not square.  Only square matrices are invertible.  Perhaps use PseudoInverse instead");
    }

    double determinant;

    try
    {
        switch (RowCount())
        {
            case 0: //error
                throw MathsException("Matrix has zero size.  Cannot find inverse.");
            case 1:
                returnValue[0][0] = 1.0 / rows[0][0];
                break;
            case 2:
                determinant = Determinant();
                returnValue[0][0] = rows[1][1] / determinant;
                returnValue[0][1] = -1.0 * rows[0][1] / determinant;
                returnValue[1][0] = -1.0 * rows[1][0] / determinant;
                returnValue[1][1] = rows[0][0] / determinant;
                break;
            default:
                returnValue = InverseByLuDecomposition();
                break;
        }
    }
    catch (const exception& e)
    {
        throw MathsException("Matrix cannot be inverted.", e);
    }

    return returnValue;
}

/* public, virtual */void Matrix::LupDecompose(Matrix& l, Matrix& u, Matrix& p) const
{
	if (!IsSquare())
	{
		throw MathsException("Matrix for LU decomposition must be square");
	}

	if (!(IsSameSize(l) && IsSameSize(u) && IsSameSize(p)))
	{
		throw MathsException("Matrices L and U must have the same dimension as the matrix to be decomposed");
	}

	double factor;
	int size = RowCount();

	u = *this; 							// Initialise the L matrix to be the same as the matrix to decompose
	l = Matrix::Zeros(size, size);		// Initialise the U matrix to zeros
	p = Matrix::Identity(size, size);	// Initialise the P matrix to the identity

	try
	{
		for (int column = 0; column < (size - 1); column++)
		{
			double biggest = fabs(u[column][column]);
			int biggestIndex = column;

			// Find the biggest candidate pivot point
			for (int row = column + 1; row < RowCount(); row++)
			{
				if (fabs(u[row][column]) > biggest)
				{
					biggest = fabs(u[row][column]);
					biggestIndex = row;
				}
			}

			// Swap biggestIndex row and "column" rows of L and U and update P matrix accordingly
			if (biggestIndex != column)
			{
				double* tempL;
				double* tempU;
				double* tempP;

				tempL = l.rows[biggestIndex];
				tempU = u.rows[biggestIndex];
				tempP = p.rows[biggestIndex];
				l.rows[biggestIndex] = l.rows[column];
				u.rows[biggestIndex] = u.rows[column];
				p.rows[biggestIndex] = p.rows[column];
				l.rows[column] = tempL;
				u.rows[column] = tempU;
				p.rows[column] = tempP;
			}

			// Perform elimination
			for (int row = column + 1; row < size; row++)
			{
				if (fabs(u[column][column]) < std::numeric_limits<double>::epsilon())
				{
					throw MathsException("Division by zero.");
				}
				factor = u[row][column] / u[column][column];
				l[row][column] = factor;
				u[row][column] = 0.0;
				for (int i = column + 1; i < RowCount(); i++)
				{
					u[row][i] = u[row][i] - factor * u[column][i];
				}
			}
		}
	}
	catch (const exception& e)
	{
		throw MathsException("Matrix is singular in LU decomposition", e);
	}

	// Add on the identity matrix to the L matrix
	l = l + Matrix::Identity(size, size);
}

/* public, static */Vector Matrix::SolveLUxEqPB(const Matrix& l, const Matrix& u, const Matrix& p, const Vector& b)
{
	// Solve equations of the form LUx = PB, where L, U and P are computed using LupDecompose()
	// This is similar to the SolveAxEqB() method but doesn't need to recompute the LUP decomposition each time it is called
	// since this is computed once only and passed into the method each time

	// Check that l, u and p are square and have the same dimension and that b has the same number of rows as p
	if (!l.IsSquare())
	{
		throw MathsException("The L matrix must be square in the SolveLUxEqPB method\n");
	}
	if (!u.IsSquare())
	{
		throw MathsException("The U matrix must be square in the SolveLUxEqPB method\n");
	}
	if (!p.IsSquare())
	{
		throw MathsException("The P matrix must be square in the SolveLUxEqPB method\n");
	}
	if (!l.IsSameSize(u))
	{
		throw MathsException("The L and U matrices must be the same size in the SolveLUxEqPB method\n");
	}
	if (!l.IsSameSize(p))
	{
		throw MathsException("The L and P matrices must be the same size in the SolveLUxEqPB method\n");
	}
	if (p.RowCount() != b.Size())
	{
		throw MathsException("The B vector must have the same number of rows as the other matrices in the SolveLUxEqPB method\n");
	}

	// Compute PB
	Vector pb = p * b;

	// Solve Ld = PB for d
	Vector d = SolveLxEqB(l, pb);

	// Solve Ux = d for x
	Vector x = SolveUxEqB(u, d);

	return x;
}

/* public, static */Vector Matrix::SolveLxEqB(const Matrix& l, const Vector& b)
{
	Vector x(l.ColumnCount());

	x[0] = b[0] / l[0][0];
	for (int i = 1; i < l.ColumnCount(); i++)
	{
		double sum = 0.0;
		for (int j = 0; j < i; j++)
		{
			sum += l[i][j] * x[j];
		}
		x[i] = (b[i] - sum) / l[i][i];
	}

	return x;
}

/* public, static */Vector Matrix::SolveUxEqB(const Matrix& u, const Vector& b)
{
	Vector x(u.ColumnCount());

	x[u.ColumnCount() - 1] = b[u.ColumnCount() - 1] / u[u.ColumnCount() - 1][u.ColumnCount() - 1];
	for (int i = u.ColumnCount() - 2; i >= 0; i--)
	{
		double sum = 0.0;
		for (int j = i + 1; j < u.ColumnCount(); j++)
		{
			sum += u[i][j] * x[j];
		}
		x[i] = (b[i] - sum) / u[i][i];
	}

	return x;
}

/* public, virtual */bool Matrix::IsSquare() const
{
	return (RowCount() == ColumnCount());
}

/* public, virtual */bool Matrix::IsSameSize(const Matrix& x) const
{
	return ((RowCount() == x.RowCount()) && (ColumnCount() == x.ColumnCount()));
}

/* public, virtual */Matrix Matrix::operator *(const Matrix& y) const
{
    return Multiply(y);
}

/* public, virtual */const char* Matrix::ToString() const
{
	char* string = new char[Size(Columns) * Size(Rows) * 20 + 2]; // This will leak unless FreeStringMemory is called on it externally
	char value[20];

	string[0] = 0;
    strcat(string, "[");
    for (int i = 0;  i < Size(Rows); i++)
    {
        if (i != 0)
        {
            strcat(string, "; ");
        }
        for (int j = 0; j < Size(Columns); j++)
        {
            if (j != 0)
            {
                strcat(string, ", ");
            }
            sprintf(value, "%g", rows[i][j]);
            strcat(string, value);
        }
    }
    strcat(string, "]");

    return string;
}

/*public, static */void Matrix::FreeStringMemory(const char* stringPointer)
{
	delete stringPointer;
}

/* public, virtual */void Matrix::Print() const
{
    printf("[");
    for (int i = 0;  i < Size(Rows); i++)
    {
        if (i != 0)
        {
            printf("; ");
        }
        for (int j = 0; j < Size(Columns); j++)
        {
            if (j != 0)
            {
                printf(", ");
            }
            printf("%g", rows[i][j]);
        }
    }
    printf("]\n");
}


    // Matrix Operators

/* public, virtual */Matrix Matrix::operator *(double y) const
{
    return Multiply(y);
}

/* public, friend */Matrix operator *(double x, Matrix y)
{
    return y.Multiply(x);
}

/* public, virtual */Vector Matrix::operator *(const Vector& y) const
{
    return Multiply(y);
}

/* public, virtual */Matrix Matrix::operator /(double y) const
{
    return Divide(y);
}

/* public, virtual */Matrix Matrix::operator +(const Matrix& y) const
{
    return Plus(y);
}

/* public, virtual */Matrix Matrix::operator -(const Matrix& y) const
{
    return Minus(y);
}

/* public, virtual */Matrix::RowType& Matrix::operator[](const int i) const
{
    if ((i < 0) || (i >= size[Rows]))
    {
        throw MathsException("Index out of range getting matrix element");
    }

    return rows[i];
}

/* public, virtual */Matrix& Matrix::operator =(const Matrix& matrix)
{
    if (this->rows != matrix.rows)
    {
        if (!this->IsSameSize(matrix))
        {
            // Must resize the array
            for (int row = 0; row < RowCount(); row++)
            {
                delete rows[row];
            }
            delete[] rows;

            this->size[Rows] = matrix.RowCount();
            this->size[Columns] = matrix.ColumnCount();

            try
            {
                rows = new RowType[matrix.RowCount()];

                for (int row = 0; row < matrix.RowCount(); row ++)
                {
                	RowType newRow;
                   	newRow = new double[matrix.ColumnCount()];
                    rows[row] = newRow;
                }
            }
            catch (const exception& e)
            {
                throw MathsException("Error allocating memory for Matrix", e);
            }
        }

        for (int i = 0; i < matrix.RowCount(); i++)
        {
            for (int j = 0; j < matrix.ColumnCount(); j++)
            {
                rows[i][j] = matrix[i][j];
            }
        }
    }

    return *this;
}

}
