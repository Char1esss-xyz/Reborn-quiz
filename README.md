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
>我最开始以为这是一个多解的问题，经过学长的点化后开悟：其实学习率设置好的话，可以得到**运算速度快**且**全局最优解**的答案。

## amour_v2:
装甲板识别，相比较v1版本其实没有很多的优化，更多的是把v1的**图片**架构转换为**视频**的架构。<br>
To do:
- [ ] 优化判断条件

## ROS2学习：
### Nodes:
Nodes负责单一、模块化的功能，类似于一个函数，可以
- 订阅数据
- 计算处理
- 发布结果

但与函数不同的是，它是一个**独立的进程**。
``` bash
# 运行一个node
ros2 run <package_name> <executable_name>

# 列出所有nodes
ros2 node list

# 重映射
ros2 run <package_name> <executable_name> --ros-args --remap __node:=name

# 查询node信息
ros2 node info <node_name>
```

### Topics
Topics起到了Nodes间数据传输的作用。
- 可以通过rqt_graph进行可视化的管理
- 可以用` ros2 topic <arguement> `进行管理

``` bash
# 列出所有topics和信息类型
ros2 topic list -t

# 查询一个topic的数据内容
ros2 topic echo <topic_name>

# 查询一个topic的信息
ros2 topic info

# 查询一个topic信息传输的数据结构
ros2 interface show <msg_type>

# 将数据推送到topic
ros2 topic pub <topic_name> <msg_type> ‘<args>’

# 查询传输频率
ros2 topic hz <topic_name>

# 查询传输带宽
ros2 topic bw <topic_name>

# 查找传输某一类数据的Topic
ros2 topic find <topic_type>
```

### Services:
Services也可以起到Nodes间传输数据的功能。<br>
与Topics不同的是，Services**仅在客户端调用时才传输数据**。
``` bash
# 列出所有services和信息类型
ros2 service list -t

# 列出service传输的信息类型
ros2 service type <service_name>

# 列出所有service传输的信息类型
ros2 service list -t

# 查询特定的service
ros2 service info <service_name>

# 查找特定类型的service
ros2 service find <type_name>

# 查询service传输的数据结构（首先要知道信息类型）
ros2 service list -t #（查询信息类型）
ros2 interface show <msg_type>

# 调用service
ros2 serive call <service_name> <service_type> <arguements>

# 查询客户端和服务端传输的数据
ros2 service echo <service_name | service_type> <arguments>
```

### Parameters:
Parameters是node的配置值。

``` bash
# 查询node的参数
ros2 param list

# 查询parameter的类型和当前值
ros2 param get <node_name> <parameter_name>

# 修改parameter的值
ros2 param set <node_name> <parameter_name> <value>

# 查询node的所有parameters
ros2 param dump <node_name>

# 加载参数到当前node
ros2 param load <node_name> <parameter_file>

# 启动node时加载参数
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

### Actions:
Action也是一种通信类型，它由三部分组成：
- Goal Service
- Feedback Topic
- Result Service

显而易见，Action是在Topic和Service的基础上产生的。但动作可以取消，还有持续的反馈。

``` bash
# 查询所有action
ros2 action list

# 查询所有的action和类型
ros2 action list -t

# 查询action的类型
ros2 action type <action_name>

# 查询action的信息
ros2 action info <action_name>

# 从命令行发送一个动作目标
ros2 action send_goal <action_name> <action_type> <values>
```