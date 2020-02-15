# simple_sim

## Usage

*simple_sim* is built based on a standard MMG method for a 7 m long KVLCC2 model. It takes inital states of the full-scale ship and control orders, i,e. the rudder angle and propeller RPS. It gives the predicted ships states after a period of simulation time which can be modified as the user indicates/

## Files
*simple_call* is an example of how to call the *simple_sim*.

## Notice

Please do not change expression in *simple_sim.m* unless you have an clear indication of what you are modifying.

## Inputs

t_start_FS：（秒）开始仿真的时间点，通常为0。
t_step_FS：（秒）仿真时间步长，通常为0.1秒，过于精细意义不大。
t_end_FS：（秒）停止仿真的时间。

x0G_FS_in：（米）基于大地坐标系，沿x轴（正北）方向的坐标。
u_FS_in：（米/秒）船舶沿船体坐标系x轴（由船中指向船艏）方向的速度。
y0G_FS_in：（米）基于大地坐标系，沿y轴（正东）方向的坐标。
v_FS_in：（米/秒）船舶沿船体坐标系y轴（由船中指向右舷）方向的速度。
psi_deg_FS_in：（度）船艏向角，船艏与正北的夹角。
r_deg_FS_in：（度/秒）船舶转首速度。

RPS_FS_in：（转/秒）螺旋桨转数
delta_deg_FS_in：（度）舵角

## Outputs

输出量为9列矩阵，行数取决于仿真时间，列数等于：(t_end_FS - t_start_FS)/t_step_FS + 1。
矩阵内各列内容与输入量基本对应，分别如下：

1. 时间序列；
2. 螺旋桨转数序列；
3. 舵角序列；
4. （米）基于大地坐标系，沿x轴（正北）方向的坐标的时间序列。
5. （米/秒）船舶沿船体坐标系x轴（由船中指向船艏）方向的速度的时间序列。
6. （米）基于大地坐标系，沿y轴（正东）方向的坐标的时间序列。
7. （米/秒）船舶沿船体坐标系y轴（由船中指向右舷）方向的速度的时间序列。
8. （度）船艏向角，船艏与正北的夹角的时间序列。
9. （度/秒）船舶转首速度的时间序列。

% output_FS_hist has 9 columns. The columns of the output are as follows:
% t_FS_out: Time series
% RPS_FS_out Time series of propeller revolution per second.
% delta_deg_FS_out: Time series of of applied rudder angels in degree.
% x0G_FS_out: Time series of x coordinate in earth-fixed system.
% u_FS_out: Time series of speed in x direction.
% y0G_FS_out: Time series of of y coordinate in earth-fixed system.
% v_FS_out: Time series of speed in y direction.
% psi_deg_out: Time series of heading angle in degree.
% r_deg_out: Time series of of yaw rate in degree.
=======
Please do not change expression in *simple_sim.m* unless you have an clear indication of what you are modifying.
