function  WayPoint = WP_2ship1(v_ts,course_ts,pos_ts,TSlength)
%% 2艘船下TS船头船尾的路径点计算，中心位置在当前位置
% 包括可变和不可变路径点
% changeLabel=0,路径点固定
% changeLabel=1,路径点不固定
% OS眼中每一个TS相对于它的目标路径点，输出为船头+船尾

    a1=2.5;  %按船头2倍船长，船尾1.5倍船长设计
    a2=2;

x0=pos_ts(1);
y0=pos_ts(2);
x1=a1*TSlength*v_ts*sind(course_ts)+x0;       %船头目标点x坐标，船头a1倍船长处
y1=a1*TSlength*v_ts*cosd(course_ts)+y0;       %船头目标点y坐标，船头a1倍船长处
x2=x0-a2*TSlength*v_ts*sind(course_ts);       %船尾目标点x坐标，船尾a2倍船长处
y2=y0-a2*TSlength*v_ts*cosd(course_ts);       %船尾目标点y坐标，船尾a2倍船长处

WayPoint=[x1 y1 x2 y2];

end


