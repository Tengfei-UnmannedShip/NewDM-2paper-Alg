%% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.A*与APF结合的第三版，A*和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
% 2.(3.0版本)添加路径点
% 3.(4.0版本)添加贝叶斯


clear all
clc;
close all
tic;%tic1

MapSize=[300,300];
GoalRange=MapSize-[15,15];
[X,Y]=meshgrid(-MapSize(1):1:MapSize(1),-MapSize(2):1:MapSize(2));
[m,n]=size(X);
%%==================================================
%环境船舶参数设置
% 基本模型参考桥墩计算方法
% ==================================================

Boat_Num=4;%船舶数量
%观察者位置
Boat(1).State(1,:)=[   0   220   180     5  30  10];  %船舶状态坐标（x,y,ang,v,l,w）ang为船舶航向 v为速度大小,l为船长，w为船宽
Boat(2).State(1,:)=[-200    50   120     5  20   6];  %船舶状态坐标（x,y,ang,v）ang为船舶航向
Boat(3).State(1,:)=[ 150    70  -120     8  10   3];
Boat(4).State(1,:)=[ 120  -150   -45    10  20   5];  %ship4是本船

for i=1:1:Boat_Num
    
    Boat(i).GoalRange=MapSize-[15,15];
    Boat(i).goal=Goal_point(Boat(i).State(1,1),Boat(i).State(1,2),Boat(i).State(1,3),Boat(i).GoalRange);
    
    Boat_eta=1;
    Boat(i).APF_factor(1)=Boat_eta;
    Boat_alfa=0.1;
    Boat(i).APF_factor(2)=Boat_alfa;
    BoatRiskFieldPeakValue=100;   %风险最大值可以根据需要随意设置
    Boat(i).APF_factor(3)=BoatRiskFieldPeakValue;
    Boat_Speed_Factor=1;        %速度方向势场衰减因子，取值越大在速度方向上影响越大
    Boat(i).APF_factor(4)=Boat_Speed_Factor;
    
end
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).State(1,1);
    Boat_y = Boat(i).State(1,2);
    Boat_theta = -Boat(i).State(1,3)/180*pi;
    Boat_Speed = Boat(i).State(1,4);
    APF_factor = Boat(i).APF_factor;
    Boat(i).APF=DrawAPF(Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize,APF_factor);
    
end

% %% APF绘图测试程序%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% APF=zeros(m,n);
% for i=1:1:length(Boat)
% APF=APF+Boat(i).APF;
%
% subplot(4,2,2*i-1)
% kk1=mesh(X,Y,Boat(i).APF);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% hold on
% plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
%
% subplot(4,2,2*i)
% kk2=contourf(X,Y,Boat(i).APF);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% hold on
% plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
%
% end
%
% figure;
% subplot(1,2,1)
% kk1=mesh(X,Y,APF);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
%
% subplot(1,2,2)
% kk2=contourf(X,Y,APF);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% A*算法开始
for i=1:1:4
    start_x = Boat(i).State(1,1)+MapSize(1);
    start_y = Boat(i).State(1,2)+MapSize(2);
    end_x = Boat(i).goal(1,1)+MapSize(1);
    end_y = Boat(i).goal(1,2)+MapSize(2);
    Astar_map=zeros(m,n);
    for k=1:1:Boat_Num
        if k~=i
            Astar_map=Astar_map++Boat(k).APF;
        end
    end
    ShipLong=4;
    Movelength=20;  %步长
    SurroundPointsNum=20; %跳整方向数，n向的A*
    valueAPF=2;  %APF势场的价值函数
    NodeOpti=0;
    [posData,courseData,courseData_deg] = AstarMain(Astar_map,start_x,start_y,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
    Boat(i).AsPos=posData;
    Boat(i).AsCourse=courseData;
    Boat(i).AsCourse_deg=courseData_deg;
end

%% A*绘制路径%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% kk2=contourf(X,Y,Astar_map);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% hold on
% plot(Boat(1).goal(1,1),Boat(1).goal(1,2),'ro','MarkerFaceColor','r');
% hold on;
% ship_icon(Boat(1).State(1,1),Boat(1).State(1,2),Boat(1).State(1,5), Boat(1).State(1,6), Boat(1).State(1,3),2 );
% 
% AstarCourse=Boat(1).Astar_course-180;
% 
% for i=1:1:length(Boat(1).Astar_pos)
%     hold on;
%     ship_icon(Boat(1).Astar_pos(i,1),Boat(1).Astar_pos(i,2),Boat(1).State(1,5)/5, Boat(1).State(1,6)/5, AstarCourse(i),1 );
% end
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc
disp(['本次运行时间: ',num2str(toc)]);
