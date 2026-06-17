---
title: "Robomaster Ekf Optimize"
date: 2026-06-07T00:02:17+08:00
draft: false
tags: ["RoboMaster", "视觉", "EKF", "调试"]
categories: ["机器人"]
summary: "记录了整理调试 RoboMaster 自瞄卡尔曼滤波器的相关知识，主要包括对卡尔曼滤波器的学习经历，理解实现"
cover: https://winter-raymond.github.io/images/xunzi2.png
math: true
---

## 背景

&emsp;&emsp;去年十月份刚到战队的时候接手了自瞄代码，当时的代码十分混乱，启动代码之后去看目标，整个EKF直接就发散了，十分诡异+猎奇，整个EKF处于一个及其诡异的状态，就是比如说观测的实际小陀螺转速是 2.5rad/s，结果观测到的收敛值可以是0.1，甚至说可能正负跳，真是活人微死了属于是。而且当时完全不知道问题，我也处于大二上学期的懵懂无知的状态，负责EKF的学长学姐也可以说是，emmm，没有很好地去解决这些问题，好在经历了后续的反复努力之后成功解决了 **EKF** + **跟踪器** 这座难啃的大山，所以这里我按照我当时遇到的问题的顺序，简单记录一下，便于后人学习。

## 机甲大师!启动！


&emsp;&emsp;虽然我们总是喜欢称之为卡尔曼滤波器，但对我而言，比起滤波器，估计器似乎是更符合其作用的名称。同时，大家也总是忽略跟踪器的真正重要作用，总是倾向于凭感觉手搓跟踪器。但是我还是那句话，要想落地实现，必须要有坚实的理论基础。

### EKF基础理论

&emsp;&emsp;俗话说得好，师傅领进门，修行在个人，网上讲EKF理论的大有人在，我实在是没必要在这里赘述一遍，这里推荐一个[华南虎的视频链接](https://www.bilibili.com/video/BV1Rh41117MT?t=13.2)，总时长很短，不到两个小时，两节课就可以看完。无需要追求什么深刻理解，而且说实话，大概率理解不了，这东西就只这样的，必须要在实战中理解才行。看完对整个过程和理论有个大致印象即可。

### 运动模型

&emsp;&emsp;这是一个大坑，我刚到组里的时候从来没有人和我讲过这个东西，后面是在优化过程中才学会的。

#### 运动模型简介
&emsp;&emsp;要实现一个良好的滤波器，除了要选择良好的滤波算法以外，如KF, EKF, UKF等等，还要选择一个好的运动模型。那，什么是运动模型？简而言之，就是一个对现实世界复杂运动情况的抽象。现实世界如此复杂，我们总要找到一种合适的模型对其进行抽象解释。比如常见的CV(Constant Velocity),CA(Constant Acceleration),CTRV(Constant Turn Rate and Velocity),Jerk等等。选择不同的模型，我们的对应的EKF具体实现形式也不同。不同的运动模型对应不同的状态向量，状态向量不同了，观测函数、状态转移函数、EKF中的状态转移函数的雅各比矩阵、UKF中的基于Sigma点集的协方差矩阵、状态噪声协方差矩阵P、过程噪声协方差矩阵Q等等，形式上都会有显著不同。前几者不必说，因为设置起来都很直观，矩阵P也不必说，这是卡尔曼滤波内部自己迭代的，无需我们在意（好吧其实这里也有几个坑，先留个伏笔，我们后面慢慢讲），主要其实还是过程噪声协方差矩阵Q，这也是最开始我遇到的第一个bug，当时写这个代码的人是用AI写的，导致书写有误，留了一个惊天大坑。下面详细介绍一下。

#### 经典运动模型Q矩阵推导示例
&emsp;&emsp;先说一下Q矩阵的物理含义。相信大家都看过这种说法：卡尔曼滤波主要融合的是观测值与预测值，通过这两个值得到一个最符合实际值的猜测。Q噪声矩阵描述的是我们对“模型预测步骤”不信任程度的数学表达，物理含义上：

1. Q 越大：越不相信模型预测，滤波器会更依赖观测数据（测量值），响应会变快但可能抖动剧烈。  
2. Q 越小：越相信模型完美，滤波器会平滑但可能对真实变化反应迟钝（滞后）。

&emsp;&emsp;接着再阐述一下其数学定义：  
&emsp;&emsp;假设系统状态方程为：

$$
x_k = Fx_{k-1} + Bu_k + w_k
$$

这里的$w_k$是过程噪声，通常假设为高斯白噪声。Q就是这个噪声的协方差：

$$
Q = E[w_k w_k^T]
$$

1. 对角线元素：每个状态变量（如位置、速度）的预测误差方差。例如  
   $Q[1,1]$ 大，意味着模型在预测“位置”时误差很大。

2. 非对角线元素：不同状态变量预测误差的相关性（通常设为零）。但是如果两个变量之间有相关性如 $x$ 和 $v_x$ 则误差相关性不为0

&emsp;&emsp;接下来，我们开始推导Q矩阵的具体表达形式（以CV模型为例）：

&emsp;&emsp;**（1）确定状态向量与状态转移矩阵**

对于一维匀速运动（CV模型），状态向量选取位置和速度：

$$
\mathbf{x}_k = \begin{bmatrix} x_k \\ \dot{x}_k \end{bmatrix}
$$

无控制输入时，离散状态转移矩阵为：

$$
F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

其中 $\Delta t$ 为采样间隔。

&emsp;&emsp;**（2）理解过程噪声的物理来源**

假设目标受到随机加速度扰动（如阵风、路面颠簸），该扰动在 $[t_{k-1}, t_k]$ 内持续作用。定义连续时间白噪声 $w(t)$ 满足：

$$
\mathbb{E}[w(t)] = 0, \quad \mathbb{E}[w(t) w(\tau)] = q_c \, \delta(t - \tau)
$$

- $q_c$：噪声强度（功率谱密度），有限常数
- $\delta(t-\tau)$：狄拉克函数，表示不同时刻噪声不相关

&emsp;&emsp;**（3）计算一个采样间隔内的位置和速度增量**

速度增量（加速度积分）：

$$
\Delta v = \int_{t_{k-1}}^{t_k} w(\tau) d\tau
$$

位置增量（速度再积分）：

$$
\Delta x = \int_{t_{k-1}}^{t_k} \int_{t_{k-1}}^{s} w(\tau) d\tau ds = \int_{0}^{\Delta t} (\Delta t - \tau) w(\tau) d\tau
$$

离散过程噪声向量为：

$$
w_k = \begin{bmatrix} \Delta x \\ \Delta v \end{bmatrix}
$$

&emsp;&emsp;**（4）计算协方差矩阵 $Q = E[w_k w_k^T]$**

需要计算三个期望值：

① 速度误差方差 $\mathbb{E}[(\Delta v)^2]$：

$$
\mathbb{E}[(\Delta v)^2] = \int_0^{\Delta t} \int_0^{\Delta t} q_c \delta(\tau - s) d\tau ds = q_c \Delta t
$$

② 位置-速度误差协方差 $\mathbb{E}[\Delta x \cdot \Delta v]$：

$$
\mathbb{E}[\Delta x \cdot \Delta v] = q_c \int_0^{\Delta t} (\Delta t - s) ds = q_c \cdot \frac{\Delta t^2}{2}
$$

③ 位置误差方差 $\mathbb{E}[(\Delta x)^2]$：

$$
\mathbb{E}[(\Delta x)^2] = q_c \int_0^{\Delta t} (\Delta t - s)^2 ds = q_c \cdot \frac{\Delta t^3}{3}
$$

&emsp;&emsp;**（5）得到CV模型的Q矩阵表达式**

$$
Q = \mathbb{E}[w_k w_k^T] = q_c \begin{bmatrix}
\dfrac{\Delta t^3}{3} & \dfrac{\Delta t^2}{2} \\[8pt]
\dfrac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

&emsp;&emsp;**（6）物理意义解读**

- $Q[1,1] \propto \Delta t^3$：位置误差随时间快速累积（位置是速度的积分）
- $Q[2,2] \propto \Delta t$：速度误差随时间线性增长
- 非对角线项不为零：位置误差与速度误差正相关——若随机加速度使速度偏大，位置也会偏大

&emsp;&emsp;**（7）工程简化形式**

实际使用中常将 $q_c$ 吸收为一个参数 $q$，记为：

$$
Q = q \begin{bmatrix}
\dfrac{\Delta t^3}{3} & \dfrac{\Delta t^2}{2} \\[8pt]
\dfrac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

&emsp;&emsp;到此，推导完毕，CA亦或者更复杂模型的对应 $Q$ 矩阵也是同理，烦请各位自行推导了，实在不会推导的，这有[一篇文章](https://zhuanlan.zhihu.com/p/1932949679668204597)，可以作为参考。

&emsp;&emsp;运动模型其实到这儿也就差不多了，那我们进入下一部分，跟踪器。

### 跟踪器

&emsp;&emsp;如果说自瞄中的检测器，滤波器，火控，即使是没有经验的人也能直观感受出来的，那跟踪器就很难直接体会到了。跟踪器主要需要实现的是数据关联，那什么是数据关联？

#### 数据关联

&emsp;&emsp;数据关联，指的是把上一帧我们追踪的目标和下一帧里的目标对齐，讲得专业一点，核心是一个在不确定性下进行决策与分配的过程，目的是解决“当前接收到的量测数据具体来自哪个目标”这一核心问题。这听上去很不直观，但其实这一流程我们的大脑也需要处理，只不过大脑不会显示地处理，一般在潜意识里就处理了。

&emsp;&emsp;举个🌰，假如说现在有一对兄妹，让你一直盯着哥哥，现在你视野里只有哥哥一个人，一眨眼，兄妹都出现在你面前了，你肯定还能找到哥哥，因为他们在外貌上有显著不同。如果现在是一对双胞胎姐妹，让你一直盯着姐姐，现在你视野里只有姐姐一个人，一眨眼，现在视野里有两个人，但是一个人和原来姐姐的位置距离10厘米，一个人和原来姐姐的位置距离20米，那么你还能定位出来姐姐的精确位置，肯定不会把妹妹误认为是姐姐，这是为什么？因为距离的连续性，只要姐姐不是超级人类应该是不会再一眨眼的时间窜出去20米的。而这就是数据关联。用代码实现这一过程就是tracker需要进行的主要工作。这一问题在理想状况下十分简单，但是当噪声较多是就是十分复杂的问题了。如何排除噪声的影响？如何让系统正确的区分噪声与真正的跳变？这是值得我们深思的问题。

&emsp;&emsp;其实如果回想卡尔曼滤波的作用，不难想到一个思路：滤波器在每一帧预测时，不仅给出了状态的预测值，还给出了这个预测的不确定性——也就是协方差矩阵 $P_{k|k-1}$。把它和观测的不确定性 $R$ 合到一起，我们就能定量地回答"这个观测离我预测的位置到底有多远、这个'远'到底正不正常"这个问题。这正是数据关联的数学抓手。

&emsp;&emsp;具体来说，定义 **新息（innovation，也叫残差）** 为观测值与预测观测值之差：

$$
\mathbf{y}_k = \mathbf{z}_k - H\hat{\mathbf{x}}_{k|k-1}
$$

它衡量的是"观测比预测偏了多少"。但光看偏了多少是不够的——偏 1cm 到底算多还是算少，取决于当前系统本身有多不确定。于是我们需要 **新息协方差（innovation covariance）** 来给这个偏差一个"尺子"：

$$
S_k = H P_{k|k-1} H^T + R
$$

&emsp;&emsp;注意这里的两项含义截然不同：$H P_{k|k-1} H^T$ 是滤波器自己迭代出来的预测不确定性，会随着收敛过程动态变化；而 $R$ 是**我们人为设定**的观测噪声协方差，它不是滤波器算出来的，恰恰是需要我们去调的参数（这一点很多人会搞混，以为置信度全是滤波器"自动给"的）。

&emsp;&emsp;有了新息和它的尺子，我们就能算出**马氏距离的平方**：

$$
d^2 = \mathbf{y}_k^T S_k^{-1} \mathbf{y}_k
$$

&emsp;&emsp;这个量的精妙之处在于：它用 $S_k$ 对新息做了归一化。系统当前越不确定（$S_k$ 越大），同样大小的偏差对应的 $d^2$ 就越小，意味着我们对这次"偏移"更宽容；反之系统已经收敛得很好（$S_k$ 很小），一点点偏移就会让 $d^2$ 飙升，提示这很可能是个误关联或跳变。

&emsp;&emsp;更关键的是，在线性高斯假设下，$d^2$ 服从自由度为观测维数的**卡方分布（$\chi^2$）**。这就给了我们一个有坚实统计意义的判据：查卡方分布表，取一个置信水平（比如 95%）对应的门限值，**$d^2$ 小于门限就接受这个观测、把它关联到当前目标；超过门限就判定为离群点，拒绝或降权处理**。这一步就是所谓的**门控（gating）**。

&emsp;&emsp;说到这里要纠正一个常见的直觉误区：并不是滤波器先验地"知道"某帧噪声大、于是给低置信度。滤波器并不知道哪个观测是噪声，它只能告诉你"在当前模型假设下，这个观测离预测有多少个标准差远"。真正做出"接受还是拒绝"决策的，是 $d^2$ 和卡方门限，而不是滤波器本身。把因果理顺了，后面调参才不会调得一头雾水。

&emsp;&emsp;当然，门控只是数据关联的第一步，它解决的是"**这个观测值不值得要**"。但在多目标、强噪声的真实场景下，往往会有多个观测同时落进同一个目标的门内，或者一个观测同时落进多个目标的门内——这时候"**到底把哪个观测分配给哪个目标**"才是真正棘手的问题。最近邻（NN）、全局最近邻（GNN）、概率数据关联（PDA/JPDA）等一系列方法，正是为了解决这个分配问题而生的。这也正是开头那个"小陀螺转速观测值正负乱跳、收敛到离谱数值"的病根所在——门控和关联一旦出错，喂给滤波器的就是错误配对的数据，再好的运动模型和 Q 矩阵也救不回来。

#### 状态机

&emsp;&emsp;这是一个很经典的算法，我在这里不赘述了。但是这不意味着，它不重要，恰恰相反，调试好一个合适的状态机至关重要，一个好的状态机可以很大地提高系统的鲁棒性，反之系统则很容易崩溃，具体需要实践中慢慢调试。

## 关于P矩阵的坑

&emsp;&emsp;伏笔回收一下，之前关于P矩阵的惊天大坑。为什么放在最后说，因为这个坑其实和状态机也脱不了干系。问题现象就是我刚开始起代码，观测一个小陀螺，效果很好，然后丢失一下目标，再去观测，效果很糟。当时我对卡尔曼滤波一知半解完全不知道为什么，就是一个找不到bug的情况。直到后来，我学习了卡尔曼滤波，回头看了我们状态机在刚看到目标时的初始化，以及进入丢失状态后的逻辑：
```cpp
if (tracker_->tracker_state == Tracker::LOST)
{
    tracker_->init(armors_msg);
    target_msg.tracking = false;
}


void Tracker::init(const Armors::SharedPtr & armors_msg)
{
    if (armors_msg->armors.empty()) {
        return;
    }

    // Simply choose the armor that is closest to image center
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto & armor : armors_msg->armors) {
        if (armor.distance_to_image_center < min_distance) {
        min_distance = armor.distance_to_image_center;
        tracked_armor = armor;
        }
    }

    initEKF(tracked_armor);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

    
    
    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;

    updateArmorsNum(tracked_armor);
}

void Tracker::initEKF(const Armor & a)
{
    double xa = a.pose.position.x;
    double ya = a.pose.position.y;
    double za = a.pose.position.z;
    last_yaw_ = 0;
    double yaw = orientationToYaw(a.pose.orientation);

    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.276;  //0.276
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    // last_z = zc, last_r = r;
    dz = za, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

    ekf.setState(target_state);
}

// Tracking state machine
if (tracker_state == DETECTING) {
    if (matched) {
        detect_count_++;
        // std::cout<<"detect_count:"<<detect_count_<<std::endl;
        if (detect_count_ > tracking_thres) {
            detect_count_ = 0;
            tracker_state = TRACKING;
        }
    } else {
        detect_count_ = 0;
        tracker_state = LOST;
    }
} else if (tracker_state == TRACKING) {
    if (!matched) {
        tracker_state = TEMP_LOST;
        lost_count_++;
    }
} else if (tracker_state == TEMP_LOST) {
    if (!matched) {
        lost_count_++;
        // std::cout<<"lost_count:"<<lost_count_<<std::endl;
        if (lost_count_ > lost_thres) {
            lost_count_ = 0;
            tracker_state = LOST;
        }
    } else {
        tracker_state = TRACKING;
        lost_count_ = 0;
    }
}
```
我们老代码的逻辑里，如果目标丢失了状态机就会从 **TRACKING** 转换到 **TEMP_LOST** 然后再到 **LOST**，再检测到的话就会先调用 `Tracker` 类的 `init()` 以及 `initEKF()` 方法，匹配成功后进入状态机 **DETECTING** 进一步进入 **TRACKING**。整体上看没什么问题。但是，注意了，我们从头到尾 `ekf` 只在 **ros2节点** 构造函数中实例化过一次。也就是说 `ekf` 这个对象的存活时间基本上是从本节点开始，到程序结束，进一步的，其成员变量是整个运行过程中是不会再初始化的，而我们的 **协方差矩阵P** 也是ekf对象的一个私有变量。意识到了吗？我们在重新观测到目标后没有初始化过 **协方差矩阵P** ，这才导致第一次观测和后面观测效果差这么多。解决方式也很简单，给`ekf`对应的类增加一个初始化 **协方差矩阵P** 的方法，在`initEKF()`方法中调用即可。后续我也是这么做的，效果很好。

&emsp;&emsp;。。。这么生艹的bug我也是没招了。好在已经解决了，还是很有成就感的。最后来一个claude大人给的结语：

## 结语

&emsp;&emsp;回过头看，从去年十月那个"活人微死"的发散 EKF，到现在能稳稳咬住小陀螺，踩过的坑其实可以归成三类：

1. **模型层面的坑**——Q 矩阵写错、运动模型选得不对。这类坑藏在数学里，现象上表现为"收敛到离谱的值"，比如转速收敛到 0.1、正负乱跳。**病根在于你喂给滤波器的"信任假设"本身就是错的。**
2. **关联层面的坑**——门控、数据关联没做好，喂进滤波器的就是配错对的数据。这类坑表现为"跳变、抽搐"，再好的模型也救不回来。
3. **工程层面的坑**——比如 P 矩阵忘了重置。这类坑和卡尔曼理论一点关系都没有，纯粹是代码生命周期的疏忽，但现象上又长得和前两类一模一样，最难定位。

&emsp;&emsp;这三类坑给我最大的教训是：**当 EKF 表现异常时，不要急着去调参数。** 先问自己三个问题——我的运动模型假设对吗（模型层）？喂进去的数据配对对吗（关联层）？对象的生命周期、状态的初始化干净吗（工程层）？很多时候疯狂调 Q、调 R 调到天昏地暗，结果病根是某个成员变量没重置，这种"在错误的层面上努力"是最消耗人的。

&emsp;&emsp;另外一点感受是，**自瞄这套东西真的没法靠"感觉"手搓。** 前期我吃的所有亏，几乎都来自"理论一知半解 + 凭直觉改代码"。直到我把卡尔曼滤波、Q 矩阵推导、数据关联的统计意义一点点啃下来，回头再看那些 bug，几乎是"一眼就知道哪里不对"。所谓"必须在实战中理解"，不是说可以跳过理论，而是说**理论给你定位 bug 的方向，实战给你确认 bug 的手感**，两者缺一不可。

&emsp;&emsp;希望这篇踩坑记录能帮到后来接手这摊代码的学弟学妹们，少走点弯路。共勉。