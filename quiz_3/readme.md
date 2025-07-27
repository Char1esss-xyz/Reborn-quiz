# 矩阵运算库
> 注：main.cpp文件中有相关测试用例及说明
## 构造函数
- 默认构造函数： Matrix()
- 维度构造函数： Matrix(rows, cols)
- 初始值构造函数： Matrix(rows, cols, value)
- 向量构造函数： Matrix(vector<vector<double>>)
- 拷贝构造函数： Matrix(const Matrix&)

## 运算符重载
- 加法重载：可通过 '+' 运算矩阵加法
- 乘法重载：可通过 '*' 运算矩阵加法
- 输出重载：可通过 '<<' 对矩阵进行打印
- 赋值重载：可通过 '=' 对矩阵进行赋值
  
## 工具函数
- at(int i, int j) &thinsp; 可查询矩阵第i行第j列的数字
- getRows() &thinsp; 可得到矩阵的行数
- getCOls() &thinsp; 可得到矩阵的列数
- isSameDimension(const Matrix &mat) &thinsp; 返回布尔变量确定两个矩阵是否维度相同
- canMultiply(const Matrix &mat) &thinsp; 返回布尔变量确定两个矩阵是否可以相乘

## 异常处理
异常情况下会丢出MatrixException类，可以通过
``` cpp
try
{
    // put your function here
}
catch (const MatrixException &e)
    {
        std::cerr << e.what() << endl;
    }
```
获取错误信息