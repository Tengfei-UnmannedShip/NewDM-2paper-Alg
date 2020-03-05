function Next_point = ang_point(x0,y0,ang,step_length)
% function goal_point = Goal_point(x0,y0,ang,range)的姊妹函数
% Goal_point是全局地图上的船找边沿处的目标点
% ang_point是以本船为中心后，找下一个步长的点
% 思路，把本船坐标转换到中点，求出来后，再转换回去

point_temp= Goal_point(0,0,ang,step_length);

x=point_temp(1)+x0;
y=point_temp(2)+y0;

Next_point=[x,y];

end

