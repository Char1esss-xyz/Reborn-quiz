#include <iostream>
#include <cmath>
using namespace std;

int main()
{
    double lower_theta, higher_theta, mid, s, time, v_0, v_0x, v_0y, k_1, z, z_actual, g, z_tmp, lr;
    bool find = false;

    // 赋值
    s = 5;
    z = 0.25;
    v_0 = 17;
    k_1 = 0.0089;
    lower_theta = 0.5 * M_PI / 180.0;                                    // 从1度开始
    higher_theta = (z >= 0) ? 89.5 * M_PI / 180.0 : 44.5 * M_PI / 180.0; // z >= 0一个解大于45度 一个解小于45度; z < 0两个解小于45度
    mid = 45 * M_PI / 180;
    lr = 0.00005;
    g = 9.8;

    // 求解小角
    while (lower_theta <= mid)
    {
        v_0x = v_0 * cos(lower_theta);
        v_0y = v_0 * sin(lower_theta);

        // 建模
        time = expm1(k_1 * s) / (k_1 * v_0x);
        z_actual = v_0y * time - 0.5 * (g * pow(time, 2));
        // 误差在0.0001内
        if (abs(z_actual - z) < 0.0001)
        {
            cout << lower_theta * 180 / M_PI << endl;
            break;
        }

        lower_theta += lr * (z - z_actual);
    }

    // 求解大角
    while ((higher_theta <= mid && z < 0) || (higher_theta >= mid && z >= 0))
    {
        v_0x = v_0 * cos(higher_theta);
        v_0y = v_0 * sin(higher_theta);

        // 建模
        time = expm1(k_1 * s) / (k_1 * v_0x);
        z_actual = v_0y * time - 0.5 * (g * pow(time, 2));
        
        // 误差在0.0001内
        if (abs(z_actual - z) < 0.0001)
        {
            cout << higher_theta * 180 / M_PI << endl;
            break;
        }

        higher_theta -= lr * (z - z_actual);
    }
    return 0;
}