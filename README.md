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
2. 【**已完成**】薛双飞的原程序与我的习惯表述是反的，row是y坐标，col是x坐标，注意每一个地方的更改，可能是mainV3中角度始终有问题的原因。
3. 【**已完成，在AstarMain函数中有初始艏向的设置**】 初始角度没有设置，因此Boat3与Boat4（mainV1/V2/V3的设置）的决策结果，一开始首先转180度从后面绕过去，这显然不合理，原因是传统的A\*算法中，起始点就是一个点，下一步的方向自己选择。要把把初始的方向也考虑进去。
4. 【**已记录几个不同的APFvalue的表现**】修改了初始艏向的设置后，Boat1的路线经过了Boat2船头，猜测是APFvalue设置（当前为2）的问题。开始做系列试验，APFvalue的值变化为\[0.1,0.5,1,2,5,10,50,100\],试验之后看最后的结果，明确变化的趋势，找到合适的值。

## 从场景到路径点
路径点的计算过程主要是：
1. 根据当前的位置、航速、航向，计算每艘船的船头船尾的两个路径点；
2. 找出对OS有避碰风险的TS，这些TS根据船头船尾点的组合，共可以形成2^TS个场景，每一个场景对应一个预期的目标点；
3. 从本船视角预计TS的未来路径（贝叶斯方法或其他），从而更新CAL，即确认下一个时刻的场景；
4. 根据新的场景，找到对应的那个目标点，并计算这个目标点和上一个目标点的距离，距离相差过远，就要重新进行A\*的路径规划。

编程细节备注：

1. 针对Boat3计算时间过长，找到了原因：第一时刻Boat3的路径点距离船本身过近，无法进行A\*算法的决策，因此，增加了路径点距离过近、路径点在本船后方时的决策————即直接将目标点设置为路径点。
2. 不管是采用综合路径点的main7还是采用单独路径点的main8，每次都会出现某几艘船一直往外走的情况。今天（2月10日）突然意识到，可能是因为我采取了只计算1000步的原因。解决方法：（1）每一次都跑完，这样就尽量减小地图的大小；（2）找到快速匹配方法。

## 路径规划算法改用FM方法（0215/2020）
FastMatching算法，又叫光程法，要求风险越大的地方值越小。此处采取了一个取巧的方法，即把APF图加1，整体取倒数，这样原来是0的地方（即无风险区域）现在为1，取倒数之后还是1，风险越大的地方值越小。

FM的程序采用了网上的FM算法工具箱。

FMmainV25是第一个可以完整运行的版本，耗时18分钟。结果存储在0220.mat中，主要问题在由于没有航向角的限制，轨迹非常奇怪。
