# NewDM-2paper-Alg
重启后新的决策论文2的算法
## 基本思路：
在本船的眼中：

1. 生成每个目标船当前的APF
2. 生成每个目标船的8个路径点
3. 贝叶斯推断给出8个路径点的预测
4. 选定对方的路径点
5. 根据3个路径点，确定自己的路径点

编程细节备注：

1. 计算中间的路径点的时候，一旦一艘船经过两船航线交点，就认为没问题了，不再考虑进去，不计算路径点
2. **【已完成】**薛双飞的原程序与我的习惯表述是反的，row是y坐标，col是x坐标，注意每一个地方的更改，可能是mainV3中角度始终有问题的原因。
3. 初始角度没有设置，因此Boat3与Boat4（mainV1/V2/V3的设置）的决策结果，一开始首先转180度从后面绕过去，这显然不合理，原因是传统的A\*算法中，起始点就是一个点，下一步的方向自己选择。要把把初始的方向也考虑进去。
