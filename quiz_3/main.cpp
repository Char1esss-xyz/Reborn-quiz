#include "include/matrix.h"
#include "iostream"

using namespace std;

int main()
{
    Matrix mat1(2, 3, 1);
    Matrix mat2(4, 4, 1);
    Matrix mat3(4, 4, 2);
    Matrix mat4(4, 5, 1);
    Matrix mat5(4, 4, 0);

    // multi matrix with wrong dimensions
    try
    {
        mat1 *mat2;
    }
    catch (const MatrixException &e)
    {
        std::cerr << e.what() << endl;
    }

    // add matrix with wrong dimensions
    try
    {
        mat1 + mat2;
    }
    catch (const MatrixException &e)
    {
        std::cerr << e.what() << endl;
    }

    // assign matrix with wrong dimensions
    try
    {
        mat1 = mat2;
    }
    catch (const MatrixException &e)
    {
        std::cerr << e.what() << endl;
    }

    // query invalid location
    try
    {
        mat1.at(3, 4);
    }
    catch (const MatrixException &e)
    {
        std::cerr << e.what() << endl;
    }

    // test isSameDimension function
    if (mat2.isSameDimension(mat3))
    {
        cout << "is same dimension" << endl;
    }
    else
        cout << "is not same dimension" << endl;

    cout << mat2 + mat3;

    // test canMultiply function
    if (mat2.canMultiply(mat4))
    {
        cout << "can multiply" << endl;
    }
    else
        cout << "can not multiply" << endl;

    // test operator *
    cout << mat2 * mat4;

    // test operator =
    cout << (mat5 = mat2);
    return 0;
}