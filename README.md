# Reborn-quiz
其实写markdown也是**学习**的一部分

## quiz 1:
一个**aplha beta滤波器**的题目，用来对测量的数据进行平滑处理，以达到拟合理想实验数据的效果。<br>
alpha beta滤波器介绍: [Wikipedia](https://en.wikipedia.org/wiki/Alpha_beta_filter)

## amour_v1:
装甲板识别，[参考代码](https://blog.csdn.net/qq_40403096/article/details/107039702)。<br>
注：只能在**amour_1.jpg**上**完美**运行，判断条件仍需完善。

## quiz 2:
一个**误差反馈的迭代优化**题目，先对子弹运动建立好模型，然后通过高度的误差进行角的迭代。<br>
>我最开始以为这是一个多解的问题，经过学长的点化后开悟原来学习率设置好是可以得到**运算速度快**且**全局最优解**的答案。

## amour_v2:
装甲板识别，相比较v1版本其实没有很多的优化，更多的是把v1的**图片**架构转换为**视频**的架构。<br>
To do:
- [ ] 优化判断条件

## ROS2学习：
### Node:
Node负责单一、模块化的功能，类似于一个函数，可以
- 订阅数据
- 计算处理
- 发布结果
  
但与函数不同的是，它是一个**独立的进程**。
``` bash
# 运行Node
ros2 run <package_name> <executable_name>

# 列出Node
ros2 node list

# 重映设
ros2 run <package_name> <executable_name> --ros-args --remap __node:=name

# 查询Node信息
ros2 node info <node_name>
```
