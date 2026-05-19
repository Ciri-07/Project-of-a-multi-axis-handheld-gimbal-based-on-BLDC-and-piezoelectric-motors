# 线性霍尔传感器近五年资料索引

本文件夹仅保留近五年左右与“线性霍尔传感器高精度位置检测、角度/位置估计、误差补偿、云台伺服与精密运动控制”直接相关的资料。

时间口径：当前为 2026 年，按年份筛选保留 2021-2026 年资料。严格按日期计算时，TIE 2021 云台论文略早于 5 年整，但它与本项目最相关，仍建议保留。

## 当前保留文件

| 文件名 | 年份 | 类型 | 题名 / 内容 | 保留原因 |
|---|---:|---|---|---|
| `01_TI_Linear_Hall_Effect_Sensor_Angle_Measurement_SLYA036B.pdf` | 2018 / 2021 修订 | TI 应用笔记 | Linear Hall-Effect Sensor Angle Measurement: Theory, Implementation, and Calibration | 不是论文，但作为线性 Hall 角度检测基础资料保留 |
| `02_Self_Calibrating_Position_Measurements_Imperfect_Hall_Sensors_2025.pdf` | 2025 | IFAC 论文 | Self-Calibrating Position Measurements: Applied to Imperfect Hall Sensors | 近五年，直接对应 Hall 传感器误差自校准与补偿 |
| `03_TIE2021_Position_Extraction_Ultralow_Speed_Gimbal_Linear_Hall.pdf` | 2021 | TIE 论文 | Position Extraction of Ultralow-Speed Gimbal Servo System With Linear Hall Sensors | 近五年边界内，且最贴近云台线性 Hall 位置提取 |
| `04_TMECH2022_3D_Printed_Halbach_Cylinder_Motor_Self_Position_Sensing.pdf` | 2022 | TMECH 论文 | A 3-D Printed Halbach-Cylinder Motor With Self-Position Sensing for Precision Motions | 近五年，体现 Hall 自位置检测和精密运动控制 |

## 已移除的旧论文

| 原文件名 | 年份 | 移除原因 |
|---|---:|---|
| `03_Mover_Position_Detection_Linear_Hall_Sensors_EKF_Sensors2017.pdf` | 2017 | 超出近五年范围 |
| `05_TMECH2012_Harnessing_Embedded_Magnetic_Fields_for_Angular_Sensing.pdf` | 2012 | 超出近五年范围 |
| `06_TMECH2010_Compact_Hall_Effect_Sensing_6DOF_Precision_Positioner.pdf` | 2010 | 超出近五年范围 |

## 资料说明

### 1. TI 应用笔记：线性 Hall 角度测量

文件：`01_TI_Linear_Hall_Effect_Sensor_Angle_Measurement_SLYA036B.pdf`

题名：Linear Hall-Effect Sensor Angle Measurement: Theory, Implementation, and Calibration

年份：2018 年发布，2021 年修订

链接：https://www.ti.com/lit/an/slya036b/slya036b.pdf

用途：

- 理解两个一维线性 Hall 传感器如何构造正余弦信号。
- 理解 `atan2` 解角、幅值校准、偏置校正、磁体布置等基础工程问题。
- 作为后续论文阅读和方案设计的基础资料。

### 2. IFAC 2025：不完美 Hall 传感器自校准

文件：`02_Self_Calibrating_Position_Measurements_Imperfect_Hall_Sensors_2025.pdf`

题名：Self-Calibrating Position Measurements: Applied to Imperfect Hall Sensors

年份：2025

DOI：10.1016/j.ifacol.2025.10.143

用途：

- 说明线性 Hall 传感器存在制造误差、安装误差和位置相关非线性时，如何通过数据驱动方式进行自校准。
- 对“无高精编码器条件下的 Hall 位置补偿”有直接参考意义。
- 可用于方案中论证“低成本 Hall 传感器 + 补偿算法”具备可行性。

### 3. TIE 2021：超低速云台线性 Hall 位置提取

文件：`03_TIE2021_Position_Extraction_Ultralow_Speed_Gimbal_Linear_Hall.pdf`

题名：Position Extraction of Ultralow-Speed Gimbal Servo System With Linear Hall Sensors

年份：2021

期刊：IEEE Transactions on Industrial Electronics

DOI：10.1109/TIE.2021.3066916

用途：

- 当前资料中最贴近手持云台项目的一篇。
- 研究对象就是超低速云台伺服系统中的线性 Hall 位置提取。
- 涉及幅值不一致、谐波干扰、信号重构和位置精度提升，适合支撑“云台线性 Hall 高精度位置检测方案”。

### 4. TMECH 2022：Hall 自位置检测精密电机

文件：`04_TMECH2022_3D_Printed_Halbach_Cylinder_Motor_Self_Position_Sensing.pdf`

题名：A 3-D Printed Halbach-Cylinder Motor With Self-Position Sensing for Precision Motions

年份：2022

期刊：IEEE/ASME Transactions on Mechatronics

DOI：10.1109/TMECH.2021.3087523

用途：

- 展示 Hall-effect sensors 在电机自位置检测和精密运动控制中的应用。
- 可作为特制微型电机中“紧凑传感器布置 + 位置反馈 + 闭环控制”的参考。
- 对后续高精度云台结构中的嵌入式位置检测有工程借鉴意义。

## 阅读建议

1. 先读 TI 应用笔记，建立线性 Hall 正余弦检测、`atan2` 解角和标定的基础概念。
2. 再读 TIE 2021 云台论文，直接学习云台低速伺服场景下的 Hall 位置提取和误差补偿。
3. 继续读 IFAC 2025，关注无外部高精编码器时如何做自校准。
4. 最后读 TMECH 2022，补充 Hall 自位置检测与精密运动控制的工程实现思路。

## 对当前项目的启发

- 线性 Hall 更适合输出正余弦两相信号，再通过 `atan2` 得到连续角度。
- 需要重点处理零偏、幅值不一致、相位不正交、磁场谐波、温漂、安装偏心、气隙变化和采样延迟。
- 只做一次解角不够，应建立“原始 Hall 电压 -> 归一化/校正 -> 解角 -> 谐波或模型补偿 -> 与 IMU/编码器融合”的完整链路。
- 对高精度云台来说，Hall 可承担高带宽相对位置反馈，IMU 或其他绝对传感器可用于低频漂移修正和姿态基准。
