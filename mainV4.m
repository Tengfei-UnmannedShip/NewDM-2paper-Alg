%% (2.0版本)扩展到海里为单位的地图
% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.A*与APF结合的第三版，A*和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
% 2.(3.0版本)添加路径点
% 3.(4.0版本)添加贝叶斯

clear
clc
close all
tic;%tic1
%% 初始设置
MapSize=[8,8];
GoalRange=MapSize-[1,1];
Res=100;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

% ==================================================
% 船舶参数设置
% ==================================================
%1~2位置(中间的位置，不是起始位置)、3航速(节)、4初始航向（deg，正北为0），5决策周期时长，6检测范围（range，nm）
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];

ShipSize = [ 250, 30
    290, 45
    290, 45
    270, 40 ];

Boat_Num=4;%船舶数量
tMax=2500;   %最大时间
tBayes=2000;    %贝叶斯推测时间
for i=1:1:Boat_Num
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground，第一行为对地速度，单位节
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %第一行为对地速度，单位米／秒
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground，第一行为初始航向（deg，正北（Y正向）为0）
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground，第一行为初始航向（rad，正北（Y正向）为0）
    %由中间位置倒推的初始位置，此处pos单位为米,随后每个时刻新增一行
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*0.5*tMax, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*0.5*tMax];
    %     Boat(i).pos_nm=Boat(i).pos/1852;    %单位为海里的pos
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    %     Boat(i).goal_nm=Boat(i).goal/1852;  %单位为海里的目标位置
    
    % APF参数设置
    Boat_eta=0.2;
    Boat(i).APF_factor(1)=Boat_eta;
    Boat_alfa=0.01;
    Boat(i).APF_factor(2)=Boat_alfa;
    BoatRiskFieldPeakValue=200;   %风险最大值可以根据需要随意设置
    Boat(i).APF_factor(3)=BoatRiskFieldPeakValue;
    Boat_Speed_Factor=1;        %速度方向势场衰减因子，取值越大在速度方向上影响越大
    Boat(i).APF_factor(4)=Boat_Speed_Factor;
    
end

%% 绘制当前每艘船的APF
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).pos(1,1);
    Boat_y = Boat(i).pos(1,2);
    Boat_theta = -Boat(i).COG_rad; %此处为弧度制
    Boat_Speed = Boat(i).speed;
    APF_factor = Boat(i).APF_factor;
    Boat(i).APF=DrawAPF(Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize*1852,APF_factor,Res);
    
    SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2 );
end

%% APF绘图测试程序%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

APF=zeros(m,n);
for i=1:1:Boat_Num
    APF=APF+Boat(i).APF;
end

figure
kk1=mesh(X,Y,APF);
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0 250])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
box off;

figure
kk2=contourf(X,Y,APF);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
% axis([xmin xmax ymin ymax zmin zmax])
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
box on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% A*算法开始
% % valueAPF0=[0.1,0.5,1,2,5,10,50,100];
%
% for i=1:1:1
%     start_x = Boat(i).pos(1,1)+MapSize(1)*1852;
%     start_y = Boat(i).pos(1,2)+MapSize(2)*1852;
%     start_theta = Boat(i).COG_rad;   %起始点艏向，弧度制
%     %A*算法的目标点，在每个时刻，添加路径点后，应该首先是路径点，经过路径点后，才是这个最终的目标点
%     end_x = Boat(i).goal(1,1)+MapSize(1)*1852;
%     end_y = Boat(i).goal(1,2)+MapSize(2)*1852;
%
%     Astar_map=zeros(m,n);
%     for k=1:1:Boat_Num
%         if k~=i
%             Astar_map=Astar_map+Boat(k).APF;
%         end
%     end
%     ShipLong=ShipSize(i,1);
%     Movelength=Boat(i).speed;  %步长,当前时刻的速度
%     SurroundPointsNum=20; %跳整方向数，n向的A*
%     valueAPF=2;  %APF势场的价值函数
%     NodeOpti=0;
%     [posData,courseData,courseData_deg] = AstarMain(Astar_map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
%
%     Boat(i).AsPos=posData;
%     Boat(i).AsCourse=courseData;
%     Boat(i).AsCourse_deg=courseData_deg;
%
% end
% % A*绘制路径%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Astar_map=zeros(m,n);
% for i=1:1:4
%     Astar_map=Astar_map+Boat(i).APF;
% end
%
% figure
% kk2=contourf(X,Y,Astar_map);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% for i=1:1:1
%     hold on
%     plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(Boat(i).pos(1,1),Boat(i).pos(1,2),ShipSize(i,1), ShipSize(i,2), Boat(i).speed,0);
%
%     for ii=1:1:length(Boat(i).AsPos)
%         hold on;
%         ship_icon(Boat(i).AsPos(ii,1),Boat(i).AsPos(ii,2),ShipSize(i,1), ShipSize(i,2), Boat(i).AsCourse_deg(ii),1 );
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc
disp(['本次运行时间: ',num2str(toc)]);
