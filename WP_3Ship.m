function waypoint = WP_3Ship( wp1,ds1,wp2,ds2 )
%% 3艘船（OS和2艘TS）的路径点计算
% 输入：每艘TS确定的一个waypoint，OS到每艘TS的距离
% 输出：waypoint，位于两个路径点wp1和wp2连线的中间
% 最终路径点到每一个路径点的距离与OS到TS的距离成正比，意义在于
% 距离近的，风险度高，路径点更贴近其本身的路径点，在这艘TS的两个不同路径点中，更容易识别。
x1=wp1(1);
y1=wp1(2);
x2=wp2(1);
y2=wp2(2);

x=(ds1*x2+ds2*x1)/(ds1+ds2);
y=(ds1*y2+ds2*y1)/(ds1+ds2);

waypoint = [x,y];

end

