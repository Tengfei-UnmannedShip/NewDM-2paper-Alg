function WP= WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel)
%% 2艘船下TS船头船尾的路径点计算，包括可变和不可变路径点
% changeLabel=0,路径点固定
% changeLabel=1,路径点不固定
% OS眼中每一个TS相对于它的目标路径点，输出为船头+船尾
% 基本的CPA计算但是多了一个航向时间，用于Jinfen的算法中计算每一阶段的CPA
% 将本船和目标船的速度由(速度值+航行角)的极坐标形式转化为(Vx,Vy)的直角坐标形式
time=1000;

dis=norm(pos_os-pos_ts);
if  changeLabel==0
    a1=2;  %按船头2倍船长，船尾1.5倍船长设计
    a2=1.5;
else
    if dis<200
        a1=200/TSlength;  %距离过近的时候按船头200米，船尾100米设计
        a2=100/TSlength;
    elseif dis>200 && dis<500
        a1=500/TSlength;  %距离过近的时候按船头500米，船尾300米设计
        a2=300/TSlength;
    elseif dis>500 && dis<1000
        a1=1;  %按船头1倍船长，船尾0.75倍船长设计
        a2=0.75;
    else
        a1=2;  %按船头2倍船长，船尾1.5倍船长设计
        a2=1.5;
    end
end

x0=pos_ts(1);
y0=pos_ts(2);
x1=a1*TSlength*sind(course_ts)+x0;       %船头目标点x坐标，船头a1倍船长处
y1=a1*TSlength*cosd(course_ts)+y0;       %船头目标点y坐标，船头a1倍船长处
x2=x0-a2*TSlength*sind(course_ts);       %船尾目标点x坐标，船尾a2倍船长处
y2=y0-a2*TSlength*cosd(course_ts);       %船尾目标点y坐标，船尾a2倍船长处

WP=[x1 y1 x2 y2];


end

