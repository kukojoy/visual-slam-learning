# 1 APE

**Absolute Pose Error (APE)**是评价位姿估计精度的常用指标，用于测量估计的相机轨迹（或位姿）与真实轨迹之间的差异。APE 常被用于评估 SLAM、VO（视觉里程计）和 VIO（视觉惯性里程计）等算法的性能。

### **定义**

给定以下数据：

- **真实轨迹**：$T_{i, gt}$，表示真实位姿（ground truth pose）
- **估计轨迹**：$T_{i,est}$，表示算法估计的位姿（estimated pose）

APE计算的是估计轨迹与真实轨迹在每个时间点的误差：
$$
APE_i=  \| T_{i, gt}^{-1} \cdot T_{i, est} \|
$$

- $T_i$ 是$i$时刻的位姿，通常表示为一个 $4 \times 4$ 的变换矩阵（包括旋转和平移）
- $\lVert \cdot \rVert$ 是误差的度量方式，通常取**F范数**（矩阵所有元素的平方和）

$$
APE =  \sqrt{\frac{1}{N} \sum_{i = 1}^N \| T_{i, gt}^{-1} \cdot T_{i, est} \|}
$$

### **计算步骤**

1. **对齐轨迹**：估计轨迹和真实轨迹需要在一个参考坐标系中对齐。得到对齐变换$\boldsymbol{T}$（本质上是估计轨迹的参考坐标系和真实轨迹的参考坐标系之间的变换）。这样能消除由坐标系差异带来的全局误差。
2. **计算误差**：对齐后，计算每个时间点位姿的相对误差（局部误差）。
3. **汇总统计**：为便于分析，通常将误差总结为：
   - 均值（Mean）
   - 中位数（Median）
   - 标准差（Std）
   - 最大值（Max）
   - 最小值（Min）
   - RMSE（Root Mean Squared Error）
   - SSE（Sum of Squared Errors）

# 2 RPE

**Relative Pose Error (RPE)**是另一种常用的轨迹评估指标，专注于测量相邻帧之间的相对位姿误差，适合评估短时间内轨迹的局部精度。RPE通常用于衡量SLAM或VO算法中短时间内的运动估计误差。

## **定义**

给定以下数据：

- **真实轨迹**：$T_{i, gt}$ 和 $T_{i + 1, gt}$，分别表示第 $i$ 和第 $i + 1$ 帧的真实位姿
- **估计轨迹**：$T_{i, est}$ 和 $T_{i + 1, est}$，分别表示第 $i$ 和第 $i + 1$ 帧的估计位姿

**RPE** 衡量在相邻帧间估计的相对运动与真实相对运动的差异，其计算公式为：
$$
REP_i = \| \Delta T_{i, gt}^{−1} \cdot \Delta T_{i, est} \|
$$
其中，
$$
\Delta T_{i,gt} = T_{i, gt}^{-1} \cdot T_{i + 1, gt}, \quad \Delta T_{i, est} = T_{i, est}^{-1} \cdot T_{i+1, est}
$$

## **计算步骤**

1. **选择时间间隔 $\Delta$**：RPE 的计算需要一个固定的时间间隔 $\Delta$，可以是1或更大的正整数。
   - $\Delta=1$时，评估的是每一帧的相邻误差。
   - $\Delta > 1$时，评估的是更长时间间隔的相对误差。
2. **计算相对运动矩阵**：对所有时间点 $i$，计算真实轨迹和估计轨迹之间的相对运动。
3. **计算误差**：对齐后，计算每个时间点 $i$ 的平移误差和旋转误差。
4. **汇总统计**：和APE类似，常用以下统计值表示RPE的结果：
   - 均值（Mean）
   - 中位数（Median）
   - 标准差（Std）
   - 最大值（Max）
   - 最小值（Min）
   - RMSE（Root Mean Squared Error）
   - SSE（Sum of Squared Errors）

# 附 evo工具包的使用

```cmd
# 1 轨迹可视化
evo_traj <data_format> <traj_1.txt> [<traj_2.txt> ... <traj_n.txt>] [--ref=<traj_gt.txt>] [-p] [--plot_mode=<mode>]

# 2 精度计算
evo_ape / evo_rpe <data_format> <traj_gt.txt> <traj_n.txt> -v -a [-p --plot_mode=<mode>] [--save_results results_n.zip]

# 3 精度对比
evo_res <results_1.zip> <results_2.zip> [--save_table results_compare.csv]

# 4 配置查看
evo_config show --brief
evo_config set A b: 设置A参数值为b
evo_config reset: 配置项恢复初始化

# 参数解释
--save_plot			: 存储绘图，后可加目录、压缩包
--save_results		: 存储结果，后可加目录、压缩包
--delta <num>		: 每隔num米统计一次误差
--delta_unit <unit>	: 增量的单位，可选参数为[f, d, r, m], 分别表示[frame, degree, rad, meter]

-r 	/ --pose_relation: 计算误差的对象
trans_part: 	平移, 单位为米(默认)
rot_part: 		旋转, 无单位(unit-less)
full: 			同时考虑旋转和平移, 无单位(unit-less)
angle_deg: 	旋转角, 单位度(degree)
angle_rad: 	旋转角, 单位弧度(rad)

-v	/ --verbose: 输出一些更详细的中间信息，如Rotation of alignment
-p	/ --plot: 启用绘图
-a 	/ --align: 采用SE(3) Umeyama对齐，只处理平移和旋转
-as / --align --correct_scale: 采用Sim(3) Umeyama对齐，同时处理平移、旋转和尺度
-s  / --correct_scale: 仅对齐尺度

data_format:  kitti, tum, euroc
mode: xyz, xz
```

