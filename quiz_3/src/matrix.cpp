#include "../include/matrix.h"
#include <iostream>

Matrix::Matrix(int rows, int cols)
{
    rows_ = rows;
    cols_ = cols;
    value_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        value_[i].resize(cols_, 0);
    }
}

Matrix::Matrix(int rows, int cols, double value)
{
    rows_ = rows;
    cols_ = cols;
    value_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        value_[i].resize(cols_, value);
    }
}

Matrix::Matrix(std::vector<std::vector<double>> srcVec)
{
    rows_ = srcVec.size();
    cols_ = srcVec[0].size();
    value_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        value_[i].resize(cols_);
        for (int j = 0; j < cols_; j++)
        {
            value_[i][j] = srcVec[i][j];
        }
    }
}

Matrix::Matrix(const Matrix &srcMat)
{
    rows_ = srcMat.rows_;
    cols_ = srcMat.cols_;
    value_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        value_[i].resize(cols_);
        for (int j = 0; j < cols_; j++)
        {
            value_[i][j] = srcMat.value_[i][j];
        }
    }
}

Matrix Matrix::operator+(const Matrix &matToAdd)
{
    if (this->rows_ != matToAdd.rows_ || this->cols_ != matToAdd.cols_)
    {
        throw MatrixException('+', *this, matToAdd);
    }

    Matrix res(this->rows_, this->cols_);
    for (int i = 0; i < rows_; i++)
    {
        res.value_[i].resize(cols_);
        for (int j = 0; j < cols_; j++)
        {
            res.value_[i][j] = this->value_[i][j] + matToAdd.value_[i][j];
        }
    }
    return res;
}

Matrix Matrix::operator*(const Matrix &matToMulti)
{
    Matrix res(this->rows_, matToMulti.cols_);

    if (this->cols_ != matToMulti.rows_)
    {
        throw MatrixException('*', *this, matToMulti);
    }

    for (int i = 0; i < this->rows_; i++)
    {
        for (int j = 0; j < matToMulti.cols_; j++)
        {
            for (int k = 0; k < this->cols_; k++)
            {
                res.value_[i][j] += this->value_[i][k] * matToMulti.value_[k][j];
            }
        }
    }
    return res;
}

Matrix Matrix::operator=(const Matrix &matToAssign)
{
    if (this->rows_ != matToAssign.rows_ || this->cols_ != matToAssign.cols_)
    {
        throw MatrixException('=', *this, matToAssign);
    }
    Matrix res(matToAssign.rows_, matToAssign.cols_);
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            res.value_[i][j] = matToAssign.value_[i][j];
        }
    }
    return res;
}

std::ostream &operator<<(std::ostream &os, const Matrix mat)
{
    for (int i = 0; i < mat.getRows(); i++)
    {
        for (int j = 0; j < mat.getCols(); ++j)
        {
            os << mat.value_[i][j] << ' ';
        }
        os << '\n';
    }
    return os;
}

double Matrix::at(int i, int j) const
{
    if (i > this->rows_ || j > this->cols_ || i < 0 || j < 0)
    {
        throw MatrixException(i, j, *this);
    }
    return this->value_[i][j];
}

bool Matrix::isSameDimension(const Matrix &mat)
{
    if (this->rows_ == mat.rows_ && this->cols_ == mat.cols_)
    {
        return true;
    }
    else
        return false;
}

bool Matrix::canMultiply(const Matrix &mat)
{
    if (this->cols_ == mat.rows_)
    {
        return true;
    }
    else
        return false;
}