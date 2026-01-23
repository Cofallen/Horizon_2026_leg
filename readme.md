## 串联腿底盘

> 正式上场之前的底盘测试，基本类似于上场。

### TODO

- [ ] 串级pid
- [ ] 旋转补偿
- [ ] 倒地自启
- [ ] 离地检测
- [ ] 打滑检测
- [ ] 上台阶
- [ ] 重心动态补偿

### 心得

> 真实大腿调起来感觉和小腿差别有很多，和仿真里也有一些不同。这里简要写一下心得，注意小腿的心得和仿真里的文档随便看看就好，里面有不少不合理之处。
>

- 底盘电机
  1. 选取的是本末关节电机，因为没有上位机的原因，写了一些函数，目前还有部分功能没有写，不过不影响控制。反馈值还差扭矩解算。
  2. 控制选择：达妙H7，注意该板很容易陀螺仪零飘以及错误中断，目前采用的是取消温控，5s测零飘，但是效果只能说能用，（TODO）。
  3. CAN分配：CAN1 髋关节电机，CAN2 轮电机+电容通信，CAN3 拨盘+上版通信。测试过程中发现fdcan效果只能说还行，控制频率不能高于1khz，否则该CAN会进入总错误中断。由于本末电机控制帧一帧即可控制四个电机，本末电机是握手式反馈，负载率比常规达妙电机要高。实际测试中选择500hz，负载率在30%左右，10分钟没有吞帧情况。


- 极性

可能是小腿的折磨时间长了（三天），我把电机极性和陀螺仪极性设定为和原小腿一样后，代码直接移植即可，直接站起来了。没有必要再反极性建模了，极性这个东西没有必要老是钻牛角尖。

当前极性：髋关节电机：逆时针为正 ，轮电机：顺时针为正。（TODO待验证）

- 零点标定

因为本末电机需要手写上位机，标零点的时候需要把CAN线全拔了，否则再接一个电池也是不太合适，所以相对于原小腿方案是把初始化数据放在了`Init`函数里。

```c
// 读取lr都应取负
ALL_MOTOR.left_front.DATA.pos_init_rad = 1.3111f;
ALL_MOTOR.left_back.DATA.pos_init_rad  = 2.9463f; 
ALL_MOTOR.right_front.DATA.pos_init_rad = -1.2931f;
ALL_MOTOR.right_back.DATA.pos_init_rad  = 2.5065f;
```

> 取负是懒得改了，因为本末电机是默认顺时针为正，取了负后顺带把这个也取了，很明显不应该。

- 初始化

要给电机使能CAN信号一定的CAN初始化时间，我这里是给了100ms的时间。

- 陀螺仪方向

注意喵板解算这里有点问题，目前我是放弃了温控，采用5s计零飘，效果只能说还行，飘大约1分钟1度，最严重的是数据会出现跳变，可能导致机体不稳定的一个因素（TODO 优化）。

另外注意一下方向：

```c
IMU->pitch=Get_Roll();//获得pitch
IMU->roll=-Get_Pitch();//获得roll
IMU->yaw=-Get_Yaw();//获得yaw
```

- LQR初始站立

随便设即可

```py
Q = np.diag([400, 1, 300, 1, 5000, 1])
R_mat = np.diag([30, 10]) 
```

站的还行。

- Pid调参

相对于小腿和仿真，明显困难的多。

```c
const float F0_control[3] = {300.0f, 1.0f, 3000.0f};
const float Yaw_control[3] = {1.0f, 0.0f, 600.0f};
const float Delta_control[3] = {20.0f, 0.1f, 50.0f};
const float Roll_control[3] = {1000.0f, 0.0f, 30.0f};

PID_init(&object->pid.F0_l, PID_POSITION, F0_control, 80.0f, 0.0f);
PID_init(&object->pid.Yaw, PID_POSITION, Yaw_control, 1.0f, 0.0f);
PID_init(&object->pid.Delta, PID_POSITION, Delta_control, 1.5f, 0.2f);
PID_init(&object->pid.Roll, PID_POSITION, Roll_control, 15.0f, 0.0f);
```

其中roll补偿的调试时间较长，很容易出现劈叉。其他的参数也较为难调。计划切换中石油的串级Pid方案。（TODO 待测试）



- 拟合K阵

也是随便设。相对于小腿不同的是，不加拟合K阵的感觉就是高腿长下很容易劈叉，但是小腿上出现的震动在该腿上没有出现（难道是LQR没有调到较为完美吗），加上拟合矩阵的效果后，使得劈叉容易调了（目前我是这样感觉的）。

- 权重调试

有别于（非常大）小腿和仿真的一个就是 **$\theta$ 的权重这里必须给很大**（目前我感觉还是不够），另外髋关节输出感觉总是小， $\phi$ 的硬度也是感觉太小，但是减小髋关节R就会直接翻车。目前这版是勉强跳跃后平衡吧。

```python
Q = np.diag([8000, 1, 800, 10, 20000, 20])
R_mat = np.diag([50, 5]) 
```

现在还是手给压力能压动 $\phi$ ，且感觉 $\theta$ 的收敛性还是不非常好。注意轮电机R应该给一个适中的值，因为机体较重。

- 跳跃测试

经过测试后，舍弃原仿真和上交方案，选取中石油方案，即将跳跃简化为三个状态。

```c
switch (state)
  {
  case idle:
    break;
  
  case compact:
    left->target.l0 -= 0.0002f;
    right->target.l0 -= 0.0002f;
    if (left->vmc_calc.L0[POS] <= 0.16f && right->vmc_calc.L0[POS] <= 0.16f)
    {
      state = flight;
    }
    break;

  case flight:
    left->target.l0 += 0.0012f;
    right->target.l0 += 0.0012f;
    left->pid.F0_l.Kp = 1000.0f;
    right->pid.F0_l.Kp = 1000.0f;
    left->pid.F0_l.max_out = 140.0f;    // 160太大，欠压16
    right->pid.F0_l.max_out = 140.0f;
    if (left->vmc_calc.L0[POS] >= 0.34f && right->vmc_calc.L0[POS] >= 0.34f)
    {
      state = retract;
    }
    break;

  case retract:
    left->target.l0 = 0.10f;
    right->target.l0 = 0.10f;
    left->limit.W_max = 0.0f;
    right->limit.W_max = 0.0f;
    if (left->vmc_calc.L0[POS] <= 0.14f && right->vmc_calc.L0[POS] <= 0.14f)
    {
      state = idle;
      left->limit.W_max = 6.0f;
      right->limit.W_max = 6.0f;
      left->pid.F0_l.Kp = 5000.0f;
      right->pid.F0_l.Kp = 5000.0f;
      left->pid.F0_l.Kd = 30000.0f;
      right->pid.F0_l.Kd = 30000.0f;
      left->pid.F0_l.max_out = 80.0f;
      right->pid.F0_l.max_out = 80.0f;
    }
    break;

  case extend:
    left->target.l0 = 0.14f;
    right->target.l0 = 0.14f;      
    left->pid.F0_l.Kd = 60000.0f;
    right->pid.F0_l.Kd = 60000.0f;
    state = idle;
    left->pid.F0_l.Kp = 6000.0f;
    right->pid.F0_l.Kp = 6000.0f;
    left->pid.F0_l.Kd = 20000.0f;
    right->pid.F0_l.Kd = 20000.0f;
    left->pid.F0_l.max_out = 80.0f;
    right->pid.F0_l.max_out = 80.0f;
    break;

  default:
    state = idle;
    break;
  }
```

注意上述代码`extend`状态下直接退出该状态（或者直接删了？），因此是等效写法。跳跃测试时先原地能稳住再说。另外，删掉伸腿是因为该Pid难调，收腿后直接伸到最大腿长再收腿，会导致失稳，索性直接去了，后也发现中石油是采用该方案，决定最终这样写。**跳跃高度取决于限幅和判断。** 

- 倒地自启

原小腿判断方案，或者简单pid收腿至稳态后起立。

- 离地检测

支持力解算平稳状态是对的，但是一旦运动即错误，猜测是加速度计噪声太大，另一阶低通滤波过于抽象，设计一个滤波器实用性不强，目前计划采用**KNN**或者**概率融合触底估计**。

- 打滑检测

三种方案：传统卡尔曼融合速度和imu加速度，需将imu加速度协方差矩阵给很大；中石油方案：一阶卡尔曼+打滑后系数K抑制输出；上交方案：自适应滤波。

先尝试传统方案和中石油方案。 另外，法老开源仿真效果似乎很好。

- 上台阶

参考中石油（也是我独立想到的），失能使能方案测试。强化学习待寒假后进行。



### 日志

- 2026-1-22 今天成功跳过了0.22m台阶，虽然姿态有点抽象。另看了中石油开源，发现比去年进度也就慢大约两周。还有就是实验室睡着真难受。
