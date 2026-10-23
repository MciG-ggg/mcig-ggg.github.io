::::quiz{title="样例卷"}
:::q{type="选择" points="2"}
OpenCV 里 `cv2.imread` 读进来的图像通道顺序是什么？

1. RGB
2. BGR
3. GBR
4. 灰度

---

答案是 **2. BGR**。转灰度要写 `cv2.COLOR_BGR2GRAY`，写成 `RGB2GRAY` 得到的灰度图红蓝互换，边缘位置会偏。

:::

:::q{type="填空" points="3"}
Canny 边缘检测由三步组成：Sobel 梯度、____________ 和双阈值滞后。

---

答案是 **非极大值抑制**。它把每像素只保留梯度方向上最大的那个，边缘因此被压成一像素宽。

:::

:::q{type="程序" points="10"}
写出用 Canny 提取边缘的代码。

---

```python
gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blurred, 50, 150)
```

:::

:::q{type="解答" points="15"}
说明 ST 图上 `DECISION: OVERTAKE` 和 `DECISION: YIELD` 的区别。

---

横轴是纵向距离 $s$，纵轴是时间 $t$。前车匀速行驶时在 ST 图上是一条直线，斜率 $\frac{dt}{ds} = \frac{1}{v}$。

DP 在可行域里搜路径。解能绕到障碍物线**上方**就是超车（OVERTAKE），解只能待在障碍物线**下方**就是让行（YIELD）。

:::
::::
