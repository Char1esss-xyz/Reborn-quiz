#include <iostream>
#include <cmath>
using namespace std;

int main()
{
    double theta, s, time, v_0, v_0x, v_0y, k_1, z, z_actual, g, z_tmp, lr;
    bool find = false;
    int round = 0;

    // 赋值
    s = 5;
    z = 0.25;
    v_0 = 17;
    k_1 = 0.0089;
    theta = atan(0.05);
    lr = 0.05;
    g = 9.8;

    // 求解
    while (theta <= 90)
    {
        v_0x = v_0 * cos(theta);
        v_0y = v_0 * sin(theta);

        // 建模
        time = expm1(k_1 * s) / (k_1 * v_0x);
        z_actual = v_0y * time - 0.5 * (g * pow(time, 2));
        
        // 误差在0.01内
        if (abs(z_actual - z) < 0.01)
        {
            cout << theta * 180 / M_PI << endl;
            cout << "round:" << round << endl;
            break;
        }

        theta += lr * (z - z_actual);
        round++;
    }

    return 0;
}