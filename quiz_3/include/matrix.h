#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <exception>
#include <sstream>
#include <string>
#include <ostream>

class Matrix
{
public:
    Matrix() {};
    Matrix(int rows, int cols);
    Matrix(int rows, int cols, double value);
    Matrix(std::vector<std::vector<double>> srcVec);
    Matrix(const Matrix &srcMat);
    Matrix operator+(const Matrix &matToAdd);
    Matrix operator*(const Matrix &matToMulti);
    Matrix operator=(const Matrix &matToAssign);
    friend std::ostream &operator<<(std::ostream &os, const Matrix mat);
    int getRows() const { return rows_; }
    int getCols() const { return cols_; }
    double at(int i, int j) const;
    bool isSameDimension(const Matrix &mat);
    bool canMultiply(const Matrix &mat);

private:
    int rows_;
    int cols_;
    std::vector<std::vector<double>> value_;
};

class MatrixException : public std::exception
{
private:
    std::string error_message_;

public:
    MatrixException(char op, Matrix mat1, Matrix mat2)
    {
        std::stringstream ss;
        ss << "error: trying " << mat1.getRows() << 'x' << mat1.getCols()
           << ' ' << op << ' '
           << mat2.getRows() << 'x' << mat2.getCols() << ", dimensions are not matched";
        error_message_ = ss.str();
    }

    MatrixException(int i, int j, Matrix mat)
    {
        std::stringstream ss;
        ss << "error: trying to query: " << '(' << i << ',' << j << ')' << ", but matrix is " << mat.getRows() << 'x' << mat.getCols();
        error_message_ = ss.str();
    }

    const char *what() const noexcept override
    {
        return error_message_.c_str();
    }
};

#endif // MATRIX_H