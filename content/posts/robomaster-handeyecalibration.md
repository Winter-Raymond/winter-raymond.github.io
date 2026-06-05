---
date: "2026-06-05T00:15:54+08:00"
draft: false
title: "RoboMaster 自瞄手眼标定调试优化记录"
cover: https://winter-raymond.github.io/images/xunzi6.png
tags: ["RoboMaster", "视觉", "调试", "手眼标定"]
categories: ["机器人"]
summary: "记录自瞄系统中手眼模块的调试过程，包括基本原理、标定手法和标定效果好坏的判断依据。"
math: true
---


## 背景

&emsp;&emsp;RMUC2026 后，尝试解决一些比赛过程中发现的问题，主要是定位到了手眼标定，原来的手眼标定和机械给的参数差得蛮多的，而且自己的静态误差也大得离谱，估计距离8m的情况下，观察高度为0.86m的目标，观测高度为1.1～1.2m，很要命的误差，后来借鉴了小鱼的标定手法，主要参考这一篇[文章](https://zhuanlan.zhihu.com/p/388965279?share_code=9boac3GKx2TA&utm_psn=2039622685618082703)，但是实际测试下来效果，怎么说呢，对比之前的超绝误差有很大进步，虽然目前还不够，但是准备买新标定板了，新标定板到了再标，感觉算法是没问题的。

## 原理

### 起源

&emsp;&emsp;了解一项技术首先要了解其起源，也就是所谓的为什么会有这项技术。所谓控制，所谓机器人，甚至所谓智能，无非两点：感知世界，进而改变世界。而对机器人来说亦是如此，感知世界靠传感器，而改变世界靠电机。而手眼标定就是为了帮助机器人更好的感知世界。感知世界的方式很多，视觉是最为重要的一种。但是在RM中，我们常用的是单目相机，这种相机感知到的世界是二维的，跟真实世界差着辈儿呢，因此，我们需要将2D坐标转化为3D，这里会用到相机内参（也需要标定）和 **PnP 解算**，也很重要，但不是我们此次的重点，所以就不赘述了。

&emsp;&emsp;通过利用上述内参标定算出的xyz是目标相对于相机坐标系的，但是**RM**中的弹道解算需要知道目标相对于我弹丸出射点的位置，这就涉及到3D坐标系的变换了，也就是需要求一个三维坐标系转换矩阵，也就是我们手眼标定要求的东西，也就是所谓的相机外参。

### 数学原理

#### 1. 坐标系定义

在手眼标定问题中，我们需要明确以下几个坐标系：

- **相机坐标系 $\{C\}$**：原点在相机光心，Z轴沿光轴方向，X、Y轴分别对应图像的水平和垂直方向
- **末端执行器坐标系 $\{E\}$**（在RM中通常是云台坐标系或枪管坐标系）：原点在云台旋转中心或弹丸出射点
- **基座坐标系 $\{B\}$**（在RM中可以是底盘坐标系或世界坐标系）：固定参考系
- **标定板坐标系 $\{T\}$**：原点在标定板的某个角点，用于提供已知的空间参考

#### 2. Eye-in-Hand 与 Eye-to-Hand

根据相机的安装位置不同，手眼标定分为两种类型：

**Eye-in-Hand（眼在手上）**：相机固连在机械臂末端或云台上，随末端执行器一起运动。这是RM中的常见配置。

此时需要求解的是**相机坐标系相对于末端执行器坐标系的固定变换**，记为 ${}^E\mathbf{T}_C$。

**Eye-to-Hand（眼在手外）**：相机固定在环境中，观察机械臂的运动。

此时需要求解的是**相机坐标系相对于基座坐标系的固定变换**。

本文主要讨论 **Eye-in-Hand** 的情况。

#### 3. 手眼标定方程推导

##### 3.1 变换关系链

假设我们让云台运动到两个不同的位姿，分别记为位姿1和位姿2。在每个位姿下，相机都能观测到同一个标定板。

对于位姿1，从基座到标定板的变换链为：

$$
{}^B\mathbf{T}_T = {}^B\mathbf{T}_{E1} \cdot {}^E\mathbf{T}_C \cdot {}^C\mathbf{T}_{T1}
$$

对于位姿2，变换链为：

$$
{}^B\mathbf{T}_T = {}^B\mathbf{T}_{E2} \cdot {}^E\mathbf{T}_C \cdot {}^C\mathbf{T}_{T2}
$$

其中：
- ${}^B\mathbf{T}_{E1}$ 和 ${}^B\mathbf{T}_{E2}$ 是云台在两个位姿下相对于基座的变换（可以通过编码器读数获得）
- ${}^E\mathbf{T}_C$ 是我们要求解的手眼标定矩阵（**固定不变**）
- ${}^C\mathbf{T}_{T1}$ 和 ${}^C\mathbf{T}_{T2}$ 是标定板相对于相机的变换（通过PnP解算获得）
- ${}^B\mathbf{T}_T$ 是标定板相对于基座的变换（**固定不变**）

##### 3.2 消元得到 $AX=XB$ 方程

由于标定板在世界中是静止的，我们可以消去 ${}^B\mathbf{T}_T$：

$$
{}^B\mathbf{T}_{E1} \cdot {}^E\mathbf{T}_C \cdot {}^C\mathbf{T}_{T1} = {}^B\mathbf{T}_{E2} \cdot {}^E\mathbf{T}_C \cdot {}^C\mathbf{T}_{T2}
$$

左右同时左乘 ${}^B\mathbf{T}_{E1}^{-1}$，右乘 ${}^C\mathbf{T}_{T2}^{-1}$：

$$
{}^E\mathbf{T}_C \cdot {}^C\mathbf{T}_{T1} \cdot {}^C\mathbf{T}_{T2}^{-1} = {}^B\mathbf{T}_{E1}^{-1} \cdot {}^B\mathbf{T}_{E2} \cdot {}^E\mathbf{T}_C
$$

令：
- $\mathbf{A} = {}^{E1}\mathbf{T}_{E2} = {}^B\mathbf{T}_{E1}^{-1} \cdot {}^B\mathbf{T}_{E2}$ （云台从位姿1到位姿2的**相对运动**）
- $\mathbf{B} = {}^{T1}\mathbf{T}_{T2} = {}^C\mathbf{T}_{T1} \cdot {}^C\mathbf{T}_{T2}^{-1}$ （标定板从观测1到观测2的**相对运动**）
- $\mathbf{X} = {}^E\mathbf{T}_C$ （待求的手眼标定矩阵）

则得到经典的 **$AX=XB$ 方程**：

$$
\mathbf{A} \mathbf{X} = \mathbf{X} \mathbf{B}
$$

这是手眼标定的核心方程。

#### 4. $AX=XB$ 方程的求解

##### 4.1 旋转与平移分离

齐次变换矩阵 $\mathbf{T}$ 可以表示为：

$$
\mathbf{T} = \begin{bmatrix}
\mathbf{R} & \mathbf{t} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

其中 $\mathbf{R} \in SO(3)$ 是旋转矩阵，$\mathbf{t} \in \mathbb{R}^3$ 是平移向量。

将 $\mathbf{A}$、$\mathbf{B}$、$\mathbf{X}$ 都展开为旋转和平移部分：

$$
\begin{bmatrix}
\mathbf{R}_A & \mathbf{t}_A \\
\mathbf{0}^T & 1
\end{bmatrix}
\begin{bmatrix}
\mathbf{R}_X & \mathbf{t}_X \\
\mathbf{0}^T & 1
\end{bmatrix}=
\begin{bmatrix}
\mathbf{R}_X & \mathbf{t}_X \\
\mathbf{0}^T & 1
\end{bmatrix}
\begin{bmatrix}
\mathbf{R}_B & \mathbf{t}_B \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

展开后得到两个独立的方程：

**旋转部分**：

$$
\mathbf{R}_A \mathbf{R}_X = \mathbf{R}_X \mathbf{R}_B
$$

**平移部分**：

$$
\mathbf{R}_A \mathbf{t}_X + \mathbf{t}_A = \mathbf{t}_X + \mathbf{R}_X \mathbf{t}_B
$$

整理平移方程：

$$
(\mathbf{I} - \mathbf{R}_A) \mathbf{t}_X = \mathbf{R}_X \mathbf{t}_B - \mathbf{t}_A
$$
 
##### 4.2 求解旋转矩阵 $\mathbf{R}_X$

旋转部分的求解通常采用 **Tsai-Lenz 方法**或 **对偶四元数方法**。

**Tsai-Lenz 方法**基于罗德里格斯公式（Rodrigues' Formula），将旋转矩阵转换为旋转轴-角表示：

$$
\mathbf{R} = \mathbf{I} + \sin\theta \cdot [\mathbf{n}]_{\times} + (1-\cos\theta) \cdot [\mathbf{n}]_{\times}^2
$$

其中 $\mathbf{n}$ 是单位旋转轴，$\theta$ 是旋转角，$[\mathbf{n}]_{\times}$ 是反对称矩阵。

从 $\mathbf{R}_A \mathbf{R}_X = \mathbf{R}_X \mathbf{R}_B$ 可以推导出：

$$
\mathbf{R}_A \mathbf{R}_X \mathbf{R}_B^T = \mathbf{R}_X
$$

这意味着 $\mathbf{R}_X$ 是矩阵 $\mathbf{R}_A \cdot \mathbf{R}_B^T$ 的不动点（eigenvector corresponding to eigenvalue 1）。

通过求解旋转轴约束方程，利用**最小二乘法**求解超定方程组得到 $\mathbf{R}_X$。


##### 4.3 求解平移向量 $\mathbf{t}_X$

在求得 $\mathbf{R}_X$ 后，将其代入平移方程：

$$
(\mathbf{I} - \mathbf{R}_A) \mathbf{t}_X = \mathbf{R}_X \mathbf{t}_B - \mathbf{t}_A
$$

这是一个关于 $\mathbf{t}_X$ 的线性方程组。利用多组观测数据（至少3组，实际中通常10-30组），构建超定方程组：

$$
\begin{bmatrix}
\mathbf{I} - \mathbf{R}_{A1} \\
\mathbf{I} - \mathbf{R}_{A2} \\
\vdots \\
\mathbf{I} - \mathbf{R}_{An}
\end{bmatrix}
\mathbf{t}_X =
\begin{bmatrix}
\mathbf{R}_X \mathbf{t}_{B1} - \mathbf{t}_{A1} \\
\mathbf{R}_X \mathbf{t}_{B2} - \mathbf{t}_{A2} \\
\vdots \\
\mathbf{R}_X \mathbf{t}_{Bn} - \mathbf{t}_{An}
\end{bmatrix}
$$

使用**最小二乘法**求解得到 $\mathbf{t}_X$。

#### 5. 优化求解

初始解可能由于测量噪声、标定板检测误差、PnP解算误差等因素存在偏差。通常会采用**非线性优化**进行精细化求解。

**代价函数**定义为所有观测下的重投影误差或变换一致性误差：

$$
\min_{\mathbf{R}_X, \mathbf{t}_X} \sum_{i=1}^{n} \left\| \mathbf{A}_i \mathbf{X} - \mathbf{X} \mathbf{B}_i \right\|^2
$$

常用的优化库包括：
- **Ceres Solver**（Google开源，支持自动微分）
- **g2o**（通用图优化框架）
- **OpenCV的calibrateHandEye函数**（集成了多种求解方法）

#### 6. 标定数据采集要求

为了保证手眼标定的精度和稳定性，数据采集需要满足：

1. **位姿多样性**：云台在不同角度、不同距离下采集10-30组数据，覆盖工作空间
2. **旋转充分性**：确保云台在Pitch和Yaw轴上都有较大幅度的旋转（避免退化配置）
3. **标定板清晰可见**：避免运动模糊、过曝、遮挡
4. **相对运动足够大**：相邻两个位姿之间的相对旋转角度建议大于15°

#### 7. RM中的简化情况

在RoboMaster实战中，如果云台的机械结构精度较高、相机安装位置相对固定，可以采用一些简化策略：

- **假设旋转对齐**：如果相机光轴与枪管方向近似平行，可以假设 $\mathbf{R}_X \approx \mathbf{I}$，仅标定平移向量 $\mathbf{t}_X$
- **实测法**：直接测量相机光心到枪管出弹口的物理偏移（X、Y、Z三个方向的距离），构造平移矩阵
- **射击验证法**：在固定距离下对准静态目标射击，根据弹着点偏差反向修正手眼参数

但这些方法精度有限，**在高水平比赛中，严谨的手眼标定仍然是必不可少的**。


## 实现

### 代码实现
&emsp;&emsp;理解原理后其实也好说，其实也好说，在有算法已经被封装好的情况下，我们需要做的其实就是获取信息。第一是标定板在相机坐标系下的位置，这个可以通过 **PnP解算** 实现，需要先进行内参标定，然后调用 **OpenCV** 的标定板识别函数即可，根据标定板的不同，调用的函数也不一样。

&emsp;&emsp;其次就是位姿相对于基座的变换了，这个可以通过，获取电控那边发来的数据得到，就是正常情况下，电控那边都会发来目前云台的 **pitch 和 yaw**，通过订阅对应的话题就可以获得当前位姿相对于基座的变换了。
这里附上部分伪代码实现：
```cpp
// 1. 检测标定板，获取相机到标定板的变换 T_C_T
cv::Mat image = camera.capture();
std::vector<cv::Point2f> corners;
bool found = cv::findChessboardCorners(image, board_size, corners);

if (found) {
    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec);
    
    // 转换为变换矩阵
    T_C_T = buildTransformMatrix(rvec, tvec);
}

// 2. 从电控获取云台姿态，构建基座到末端的变换 T_B_E
float pitch, yaw;
gimbal_topic.subscribe([&](auto msg) {
    pitch = msg.pitch;
    yaw = msg.yaw;
});

// 根据 pitch 和 yaw 构建旋转矩阵
Eigen::Matrix3d R_yaw = rotationZ(yaw);
Eigen::Matrix3d R_pitch = rotationY(pitch);
Eigen::Matrix3d R_B_E = R_yaw * R_pitch;

T_B_E = buildTransformMatrix(R_B_E, t_base_to_gimbal);

// 3. 收集多组数据
std::vector<Eigen::Matrix4d> A_list, B_list;
for (int i = 0; i < num_poses; ++i) {
    capture_calibration_data();
    
    // 计算相对变换
    A_list.push_back(T_B_E[i].inverse() * T_B_E[i+1]);  // 云台相对运动
    B_list.push_back(T_C_T[i].inverse() * T_C_T[i+1]);  // 标定板相对运动
}

// 4. 调用 OpenCV 手眼标定函数求解 X
cv::Mat R_cam2gripper, t_cam2gripper;
cv::calibrateHandEye(R_gripper2base_list, t_gripper2base_list,
                     R_target2cam_list, t_target2cam_list,
                     R_cam2gripper, t_cam2gripper,
                     cv::CALIB_HAND_EYE_TSAI);

// 得到最终的手眼标定矩阵 T_E_C
Eigen::Matrix4d T_E_C = buildTransformMatrix(R_cam2gripper, t_cam2gripper);
```
基本原理就是这样，具体实现需要根据自己的情况改变。

### 精度提升技巧

&emsp;&emsp;依旧，了解原理后其实就可以 get 到主要的误差来源了：  
1. 识别误差：识别角点有误差导致 $\mathbf{B}$ 计算有误差  

2. 电控误差：电控发过来的 pitch 和 yaw 和真实值对比有误差，导致 $\mathbf{A}$ 计算有误差（就比如我们的电控之前发来的值是IMU的值，还飘着呢）  

3. 计算误差：最后最小二乘计算时误差较大  
    
了解清楚之后，其实我们也可以对症下药了。  
减小识别误差就要用精度更高的标定板，同时标定板尽可能离相机近一点，也可以在算法上实现，减小重投影误差较小的点：
```cpp
    // 重投影误差检查：过滤质量差的帧
    // centers_3d_ 是根据标定板参数和2D角点求出来的3D角点坐标
    // camera_matrix_ 是相机内参矩阵
    // distort_coeffs 是相机畸变系数
    std::vector<cv::Point2f> projected;
    cv::projectPoints(centers_3d_, rvec, tvec, camera_matrix, distort_coeffs, projected);
    double total_err = 0.0;
    for (size_t k = 0; k < projected.size(); k++) {
      total_err += cv::norm(projected[k] - centers_2d[k]);
    }
    double mean_err = total_err / projected.size();
    if (mean_err > 1.0) {
      RCLCPP_WARN(logger, "[rejected] %s 重投影误差过大: %.3f px", img_path.c_str(), mean_err);
      continue;
    }
    RCLCPP_INFO(logger, "[accepted] %s 重投影误差: %.3f px", img_path.c_str(), mean_err);
```

减小电控误差就让电控把编码器的值发过来，同时两个位姿之间的移动角度尽可能大一点，减小编码器抖动的误差的影响。

减小计算误差就让采样尽可能多，同时可以利用重投影误差去除识别查的点。

## 效果

### 效果检验

&emsp;&emsp;这个更简单了，保证我方机器人与目标位置固定不变，始终保证我方机器人观测到目标的情况下，上下左右晃机器人的云台，观测到的目标坐标与位姿波动越小，标定得越好。

### 效果分享

&emsp;&emsp;留个白，等新标定板到了再写，并对比一下。

## 总结

&emsp;&emsp;也留个白。

---

*装备：RoboMaster 英雄，HikVision 12mm 工业相机，Intel*