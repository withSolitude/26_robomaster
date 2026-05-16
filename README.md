# 26赛季代码
神秘代码

伴随着失望与不甘，是时候与rm说结束了。队内摆烂风气成灾，不思进取，不懂技术空谈的人比比皆是，甚至有些人长时间看不到人，找借口硬拖工作，所有的工作全部压在少数认真干活的“老实人”身上，我实在看不到这样的队伍的希望。在这里上传一套代码，希望能够给后来的同学一些帮助。该代码仅供参考使用。

代码主体参考中科大麻神开源电控框架，我的工作大多都是对麻神的拙劣模仿，同时根据学校硬件的不同，修改了一下底层的驱动，如电机等部分代码，同时重新模仿编写了上层控制逻辑，新增了vofa绘图等调试工具代码，vt13图传，达妙imu，上下板通信逻辑等代码。
同时感谢湘潭大学谭恩泽在板载imu方面做的工作和香港科技大学的功率控制开源。

infantry是赛季我临时接管重新编写的代码，主体是从hero移植而来，并做了一部分修改和删减，想要看代码的友友看hero就可以了。

hero包括两套代码，其中chassis是下板代码，主要负责裁判系统数据接收并与传输至上板，gimbal是上板代码，整车控制都由gimbal完成。
两个工程都采用了类似的五层目录结构，核心用户代码集中在 `User_File` 下。

```text
User_File
├── 1_Middleware
│   ├── 1_Driver       # CAN / UART / SPI / TIM / ADC / USB 等底层驱动封装
│   ├── 2_Algorithm    # PID、斜坡、滤波、RLS、队列、FSM 等通用算法
│   └── 3_System       # 时间戳等系统工具，上板工程中使用
│
├── 2_Device           # 设备层：电机、IMU、遥控器、裁判系统、VOFA、自瞄等
│
├── 3_Robot            # 机器人子系统层：Chassis / Gimbal / Shoot / 上下板通信
 
├── 4_Interaction      # 整车交互层：Class_Robot，统一组织状态机和控制逻辑
│
└── 5_Task             # 任务层：Task_Init、Task_Loop、外设回调、定时器调度
```

底盘控制采用的是力控前馈加轮速速度环的闭环控制流程，可以实现较好的底盘抗扰性，但是在斜坡补偿方面还可以通过加入底盘imu监测角度并做出相应补偿从而实现坡上平稳，同时加入了腿关节电机控制和履带驱动，腿关节电机控制建议参考infantry部分使用达妙电机的mit控制或者一拖四模式，控制效果较位置速度模式更好，功率控制部分参考香港科技大学的功率控制开源，采用RLS去电机功率模型的k1,k2值，从而起到更好的适应性，但是由于前期没有合适的功率计，此功能并未正式上车，只保留了功率预测与直接对预测值的限制，后续建议加入自动上台阶状态机，通过imu数据实现底盘较地面水平的自动上台阶控制，云台控制也是使用达妙电机的mit模式控制，以imu做位置外环，电机反馈速度做速度内环，同时加入重力补偿，角速度前馈等优化，发射部分逻辑有待优化，加入了卡弹策略有限自动机，实现卡弹自动反拨，防止长时间卡弹硬拨对电机和结构造成损伤。


由于大部分调试数据没有保存，只上传少数截图。
yaw轴相应曲线
<img width="1583" height="932" alt="屏幕截图 2026-05-04 054820" src="https://github.com/user-attachments/assets/b63140d6-5326-4cae-b3e9-821f7a111067" />
拨弹电机曲线
<img width="1057" height="464" alt="屏幕截图 2026-05-11 055739" src="https://github.com/user-attachments/assets/202c48c5-90f8-4f8c-9a56-0c4dd982ec3b" />


赛季照片：
<img width="1440" height="875" alt="微信图片_2026-05-17_022452_943" src="https://github.com/user-attachments/assets/bbcab43f-f298-43b3-a43a-d888ec70c0da" />
<img width="1920" height="1280" alt="微信图片_20260517023127_102_1" src="https://github.com/user-attachments/assets/c3419111-6ee6-46fb-a3e5-c66ea3983b0d" />
<img width="1440" height="1080" alt="微信图片_2026-05-17_024411_399" src="https://github.com/user-attachments/assets/c93694ad-8eb0-412f-9405-4581b331fba7" />
<img width="1920" height="1280" alt="微信图片_20260517023128_103_1" src="https://github.com/user-attachments/assets/21af6af9-4b58-45ea-be6f-e3f1768f10d6" />
<img width="1440" height="960" alt="微信图片_2026-05-17_022312_269" src="https://github.com/user-attachments/assets/128f24bb-b2a7-4c06-9acc-0155332ac1c8" />
<img width="1280" height="1707" alt="微信图片_20260517022622_100_1" src="https://github.com/user-attachments/assets/6e4836d8-a657-4d4b-b91f-9ca73e950bf5" />
<img width="1920" height="1280" alt="微信图片_20260517023126_101_1" src="https://github.com/user-attachments/assets/8cb1a9c1-e46d-4278-81de-999bcaa0ad63" />
<img width="1440" height="1080" alt="微信图片_2026-05-17_022423_323" src="https://github.com/user-attachments/assets/1684db59-cb69-43c8-8b9e-3b27ef96fe6c" />
<img width="1280" height="1920" alt="微信图片_2026-05-17_022335_039" src="https://github.com/user-attachments/assets/eb0448f2-638b-40a6-a5c6-4f4e6412ad9e" />
