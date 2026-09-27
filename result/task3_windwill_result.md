# 任务三：识别与稳定跟踪报告

## 素材与运行

- task_3.mp4：小能量机关场景，1440×1080，30 FPS，796 帧。
- task_4.mp4：大能量机关场景，1440×1080，30 FPS，1800 帧。
- 方法：OpenCV HSV 分割、轮廓筛选、R 标模板匹配及角度/半径关联，不使用拟合算法。
- 运行命令:
```bash
./build_task3/task3_windmill resourses/task_3.mp4 config/task_3.yaml task_3
./build_task3/task3_windmill resourses/task_4.mp4 config/task_4.yaml task_4
```


## 识别方法

HSV H=0～30 或 170～179，S≥100，靶环 V≥20；3×3 闭运算后按面积 1500～9000、短长边比≥0.65、圆度≥0.40、至少 3 个有效子孔洞筛选。候选距 R 标 100～300 像素，并要求径向灯带命中比例≥0.40。R 标匹配门限为 0.93，逐帧更新位置。

task_3 用第 216 帧 (738,501,32,32) 提取模板；task_4 用第 0 帧 (775,742,32,32)。均从第 0 帧完整处理。

## 锁定与重选规则

首次从有效候选中选最左者并分配新 ID。已锁定时先按相对 R 标的角度预测与轨道半径匹配旧目标。基础角度门限 0.30 rad，按失配帧间隔适度扩大但不超过 0.55 rad；相对半径差≤0.25。真实视频的帧间变化只用于关联，不将播放时间解释为真实运动时间。

失败帧标为 lost；task_3 连续失配 8 帧、task_4 连续失配 15 帧后释放 ID，下一帧允许新选择。重选分配新 ID。短暂 lost 不立即更换目标。

## C++ 实际结果

| 输入 | 实际输出帧数/FPS | 是否正确标注 R 标与靶环 | 双目标身份保持检查 | 失败片段 |
|---|---|---|---|---|
| task_3 | 待填 | 待填 | 待填 | 待填 |
| task_4 | 待填 | 待填 | 待填 | 待填 |

检测帧比例不能当作准确率。已知局限包括暗环断裂、灯带熄灭导致候选消失、R 标尺度/姿态变化导致匹配失败、角度关联可能误选；请填写实际观察到的帧号与原因。

## 输出链接

- [task_3 识别视频](task3_windmill/task_3/recognition_overlay.mp4)
- [task_3 二值视频](task3_windmill/task_3/binary_process.mp4)
- [task_4 识别视频](task3_windmill/task_4/recognition_overlay.mp4)
- [task_4 二值视频](task3_windmill/task_4/binary_process.mp4)
