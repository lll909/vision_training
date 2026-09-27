#Robomaster 视觉培训 - 统一工程
##一，项目简介
本项目为算法组第二次培训的统一工程，包含OpenCV图像处理，Eigen/Ceres 参数拟合以及真实能量机关识别与跟踪三个任务

##二，环境信息
- Ubuntu 24.04.4 LTS
- GCC 13+
- CMake 3.28+
- OpenCV 4.6.0
- Eigen 3.4
- Ceres Solver 2.1.0


##三，目录结构
- `CMakeLists.txt`:工程的构建配置文件
- `README.md`:项目总说明书
- `.gitignore`:git忽略规则（排斥build等生成物）
- `src/`: 所有源代码
- `src/task1_image/main.cpp`:任务一的源代码（OpenCV图像处理）
- `src/task2_fit/main.cpp`:任务二源代码（视频拟合）
- `src/task3_windmill/main.cpp`:任务三源代码（稳定跟踪）
- `resources/`:输入素材（图片和视频）
- `result/`:所有输出结果
- `result/task1_image/`:任务一所有处理后的结果图（16张）
- `result/task2_fit/`:任务二结果视频与图表
- `result/task3_windmill/`:任务三结果视频
- `result/task2_fit_result.md`:任务二详尽技术说明
- `/result/task3_tracking_result.md`:任务三详尽技术说明

##四，构建与运行
在项目根目录下，依次执行以下命令：
```bash
cmake -S . -B build
cmake --build build -j2
./build/task1_image
./build/task2_fit
./bukid/task3_windill
```

##五，任务一：OpenCV图片处理分析
1. 图像读取与颜色空间转换
  成功读取原图并检查了是否为空
  提取了灰度图并分离了HSV的三个通道（H，S，V，对应结果图）： gray.png, hsv_h.png, hsv_s.png, hsv_v.png
2. 图像滤波对比
  均值滤波（核尺寸5*5）：平滑效果很明显，但是花瓣边缘变得最模糊
  高斯滤波（核尺寸5*5，sigmaX=1.5）：平衡效果
  中值滤波（核尺寸5）去除孤立噪点的效果最好，同时保留了较为清晰的花瓣边界
  对应结果图：mean_filter.png, gaussian_filter.png, median_filter.png
3.  红色提取
  使用HSV颜色空间空间对红色进行提取，因为红色横跨了H区间的首尾两端，所以使用双区间
  低区间：Scalar(0,100,100) 到Scalar（10,255,255）
  高区间：Scalar（170,100,100）到Scalar（179,255,255）
  提取后发现：黄色边缘被精准过滤，但由于花瓣的反光或者阴影区域由于饱和度过低（S<100），存在部分检漏
  对应结果图：red_mask.png
4. 形态学操作和轮廓筛选
  使用5*5的矩形元素（kernel）进行处理
  腐蚀与膨胀：分别展示，看到目标边缘缩小和扩大。对应erode.png ,dilate.png
  开运算：去除背景中孤立的小白噪点。对应open.png
  闭运算：填补花瓣中间的小黑洞和连接断开的间隙。对应close.png
  轮廓筛选：使用findContours提取外轮廓，按面积筛选（面积>=500）,过滤掉了噪点和过小的红色区域。最后用rectangle画出外界矩形，用drawCoutours画出绿色轮廓,并且在矩形左上角标注了面积。对应contours.png
5. 绘制与几何变换
  在图像副本上绘制了蓝色的图，黄色的矩形和白色的文字。对应：drawing,png
  使用getRotationMatrix2D和warpAffine经图像绕中心你时间旋转35度。对应rotated_35deg.png
  使用React裁减出原图右上角1/4的区域。对应crop_top_left.png

##6,任务二：合成视频参数拟合
**核心目标**：从现成视频中识别青色圆点，计算角速度并且利用Ceres Solver拟合参数模型：omega（t）=b+A sin(Omega(t)+phi).
**核心方法**：图像矩求心，角度展开，移动平均滤波，Huber Loss降噪，非线性最小二乘拟合。
**详细分析报告**：请查看[任务二详细说明](result/task2_fit_result.md).
**运行方式**：
```bash
./bulid/task2_fit
```
**结果清单**：
观测与拟合对比图： result/task2_fit/fit_comparison.png
角速度估计图: result/task2_fit/angular_velocity.png
残差图: result/task2_fit/residuals.png
带识别标记的视频： result/task2_fit/tracking_overlay.mp4

##7,任务三：真实能量机关稳定跟踪
对两个真实比赛视频进行靶环识别与稳定锁定，使用 HSV 颜色提取 + 轮廓形状筛选 + R标模板匹配 + 角度预测关联 + 丢失容忍状态机。
**主要数据**
· 颜色范围：红橙色，H=0~30 或 170~179，S≥100，V≥20。
· 形状筛选：面积 1500~9000，短长边比 ≥ 0.65，圆度 ≥ 0.40，至少 3 个有效子孔洞。
· 候选半径：距 R 标 100~300 像素，径向灯带命中比例 ≥ 0.40。
· 锁定规则：首次选最左候选；已锁定时按角度和半径预测匹配；连续丢失 8 帧（task_3）/15 帧（task_4）后释放 ID，允许重选。

输出视频与日志位于：

· result/task3_windmill/task_3/
· result/task3_windmill/task_4/

详细报告见  [任务3详细说明](result/task3_windwill_result.md)。

##8,作者
姓名：王子凡
学号：2264215100
时期：2026.9.27
- 备注：任务三在完成第一版后整体效果很烂, 感觉超出能力范围，故丢给gpt完成改善和拓展，但是主要骨架是自己搭建。前两个任务是在ai辅助下手敲的。
