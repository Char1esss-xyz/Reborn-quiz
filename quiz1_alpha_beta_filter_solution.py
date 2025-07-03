import matplotlib.pyplot as plt

z_n = [30171, 30353, 30756, 30799, 31018, 31278, 31276, 31379, 31748, 32175]
true_value = [30200, 30400,	30600, 30800, 31000, 31200,	31400, 31600, 31800, 32000]
x_n = []
v_n = []
x_n1 = []
v_n1 = []
dt = 5
x0 = 30000
v0 = 40

alpha = 0.2
beta = 0.02

for x in z_n:
    forward_x = x0 + (v0 * dt) # 预测路程
    forward_v = v0 # 预测速度
    residual = x - forward_x # 计算残差
    forward_x += alpha * residual # 路程加权
    forward_v += beta * residual / dt # 速度加权

    x0 = forward_x
    v0 = forward_v

    x1 = x0 + (forward_v * dt)
    v1 = v0

    x_n.append(forward_x)
    v_n.append(forward_v)

    if(x == 30171):
        x_n1.append(forward_x)
        v_n1.append(forward_v)
    
    if(x != 32175):
        x_n1.append(x1)
        v_n1.append(v1)

time_steps = range(1, len(z_n) + 1)

plt.figure(figsize=(16, 10))

plt.plot(time_steps, z_n, 'o--', label='Measurements')
plt.plot(time_steps, true_value, 'g-', linewidth=2, label='True values')
plt.plot(time_steps, x_n, 'r-^', label='Estimates')
plt.plot(time_steps, x_n1, 'm:', label='Predition')

plt.title('Range vs. Time', fontsize=16)
plt.xlabel('Step(5s/step)', fontsize=12)
plt.ylabel('Range(m)', fontsize=12)

plt.legend()
plt.grid(True)

plt.savefig('alpha_beta_filter_comparison.png')

plt.show()

print(x_n)
print(v_n)
print(x_n1)
print(v_n1)