## 变量解释
- diff_result: 帧间差分结果
- diff_temp: ？
- drone_pos: 
- greyFrame: src1的灰度图
- image2: 带矩形框和中心点的src1 & 还用于极坐标范围显示
- img1: 极坐标图像二值化
- img2: 
- img22: 极坐标二值图向上遍历找到边界的结果
- imgg: 极坐标图像 
- outImage: 等同于src1，作用？
- src1: 前一帧图像
- src2: 后一帧图像

## 大概步骤
1. 帧间差分，得到无人机运动目标
2. 投影法绘制矩形框，确定矩形中心
3. 转为极坐标系，向上遍历确定轮廓，然后拟合出完全轮廓
4. 找8个曲率最大的点，计算曲率/r的平均值，取大于平均值的点作为要害点

