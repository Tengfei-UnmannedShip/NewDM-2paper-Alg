%% A*与APF结合的第二版，A*在主函数，起始点根据本船（ship4）设置变化
% A*算法原版作者薛双飞
% 解决的第一版程序中存在的问题：
% 0.没有中间点??中间点应该是在APF地图画好之后的，所以，APF应该也是一个函数
% 1.地图是反的??坐标转换
% 2.颜色为黑白-是否把路径取出来放到等高线图里显示，参考classicAPF/new-main里的方法

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
Boat_Speed_Factor=1;        %速度方向势场衰减因子，取值越大在速度方向上影响越大
BoatRiskFieldPeakValue=100;   %风险最大值可以根据需要随意设置
Boat_eta=1;
Boat_alfa=0.1;
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue，即每一点的势场值。此处先将其归零为一个与X相同大小的0矩阵

U_att=zeros(m,n);
APFValue=zeros(m,n);

Boat_Num=3;%船舶数量
%观察者位置
Boat.State(1,:)=[   0   220   180     5  30  10];  %船舶状态坐标（x,y,ang,v,l,w）ang为船舶航向 v为速度大小,l为船长，w为船宽
Boat.State(2,:)=[-200    50   120     5  20   6];  %船舶状态坐标（x,y,ang,v）ang为船舶航向
Boat.State(3,:)=[ 150    70  -120     8  10   3];
Boat.State(4,:)=[ 120  -150   -45    10  20   5];  %ship4是本船
% Boat.State(4,:)=[ -70   -15    90    10  20   5];

goal=Goal_point(Boat.State(4,1),Boat.State(4,2),Boat.State(4,3),GoalRange);
k_near=10;%计算目标点附近引力需要的增益系数
k_far=10; %计算引力门槛之后引力恒定时的增益系数，一般与k_near相同，否则在门槛处会出现引力场突变
Po_att=100;%引力门槛半径
k=0.3;%引力系数
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);

for i=1:1:Boat_Num

    %航标参数初始化
    Boat_x(i)=Boat.State(i,1);                  %第i个船x坐标
    Boat_y(i)=Boat.State(i,2);                  %第i个船y坐标
    Boat_theta(i)=-Boat.State(i,3)/180*pi;       %第i个船艏向角度
    Boat_Speed(i)=Boat.State(i,4);              %第i个船速度大小
    Boat_length(i)=Boat.State(i,5);             %第i个船长
    Boat_width(i)=Boat.State(i,6);              %第i个船宽
    
    %把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX{i},BoatY{i}）
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    BoatSpeedFactor{i}=Boat_Speed_Factor*Boat_Speed(i);
    
    %计算空间中点到第i个船边沿距离Dis{i}
    Dis{i}=zeros(m,n);
    Dis{i}=sqrt(BoatX{i}.^2+BoatY{i}.^2).*(BoatY{i}<=0)+sqrt((BoatY{i}/(Boat_Speed(i)+1)).^2+BoatX{i}.^2).*(BoatY{i}>0);
    
    %计算第i个船风险场
    BoatRiskField{i}=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis{i})./(Boat_alfa*Dis{i}));
    
    if  BoatRiskField{i}>BoatRiskFieldPeakValue
        BoatRiskField{i}=BoatRiskFieldPeakValue;
    end
    BoatRiskField{i}(BoatRiskField{i}>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;
    
    if BoatCut==1
        BoatRiskField{i}=BoatRiskField{i}.*(BoatX{i}>=0)+0.*(BoatX{i}<0);
    end
    
    %这里每个点在不同船下的场之间暂采用简单加和
    RiskFieldValue=RiskFieldValue+BoatRiskField{i};
end
% APFValue=max(U_att,RiskFieldValue);
APFValue=RiskFieldValue;
newfield=RiskFieldValue/BoatRiskFieldPeakValue;
figure;
kk1=mesh(X,Y,APFValue);
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),1 );
axis equal
axis off
%     surf(X,Y,APFValue);

figure
% kk2=pcolor(APFValue);
kk2=contourf(X,Y,APFValue);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
% set(kk2, 'LineStyle','none');
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),1 );
% axis equal
% axis off



%% A*算法开始
% function [SetClose,SetOpen]=CircleAStar(map,start_y,start_x,end_y,end_x)
start_x = Boat.State(4,1)+300;
start_y = Boat.State(4,2)+300;
end_x = goal(1,1)+300;
end_y = goal(1,2)+300;

map=newfield;

%% 算法5:多向A*算法
%输入: 惩罚图像(矩阵)map,起点图像坐标(start_y,start_x),目标点图像坐标(destination_y, destination_x),船舶长度ShipLong,旋回半斤Rmin
%输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表
%% line1. 设置初始船舶艏向；
background=map;
start_point.y=start_y;
start_point.x=start_x;
destination.y=end_y;
destination.x=end_x;
ShipLong=4;
Movelength=20;  %步长
SurroundPointsNum=20; %跳整方向数，n向的A*
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %转弯半径
valueAPF=2;  %APF势场的价值函数

%开始计算
%% line2. 初始准备：
%如果船舶位置在地图范围之外或者船舶状态不安全，算法结束，提示无安全路径，
%否则计算起始节点各属性(坐标、艏向、惩罚值、移动代价G、到目标点的预计代价H、总代价F、下一步移动距离r、父节点、子节点等)
%并将该节点放到open表中,初始化close列表为空；
if (0<start_point.x<length(background(1,:))&&0<start_point.y<length(background(:,1)))
    start_point.G=0; %移动代价 G
    start_point.H=sqrt((destination.x-start_point.x)^2+(destination.y-start_point.y)^2);  %到目标点的预计代价H
    start_point.F=start_point.G+start_point.H; %总代价F
    start_point.R= Movelength; %下一步移动距离r
    start_point.Dir=pi/2;  %起始点艏向
    SetOpen(1)=start_point; %起始点坐标
    SetOpen(1).father=nan; %父节点
    SetClose(1)=SetOpen(1); %并将该节点放到open表中,初始化close列表为空；
end
%% 开始计算
while  ~isempty(SetOpen)  %line3.While: open 列表不为空
    for ii=2:length(SetOpen)  %line4.寻找open列表中F值最小的节点，记为FMin；
        if SetOpen(ii).F < SetOpen(1).F
            a=SetOpen(ii);
            SetOpen(ii)=SetOpen(1);
            SetOpen(1)=a;
        end
    end
    SetClose=[SetClose;SetOpen(1)]; %line5-1.将FMin加入close列表,所以FMin就是SetClose(end),同时在open列表中删除该点；
    SetOpen(1)=[]; %line5-2.将FMin加入close列表，同时在open列表中删除该点；
    Surround=[];
    
    %         %% 算法4：邻近节点优选
    %         %输入：A*算法中的close列表
    %         %输出：优化后的close列表
    %         %%%回馈处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         L_Close=length(SetClose);
    %         ComPoint=[];
    %         if L_Close>2
    %             ComPoint=SetClose(end).father;
    %             while ~(ComPoint.y==start_y && ComPoint.x==start_x)
    %                 if ((SetClose(end).y-ComPoint.y)^2+(SetClose(end).x-ComPoint.x)^2)<(ComPoint.R)^2
    %                     SetClose(end).father=ComPoint;
    %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.y,ComPoint.x);
    %                 end
    %                 ComPoint=ComPoint.father;
    %             end
    %         end
    %         SetClose(end).father=ComPoint;
    
    %% %%变步长设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. 根据 FMin 节点的惩罚值大小计算下一步应该移动的步长 ShipSpeed；
    ShipSpeed=Movelength * (1-map(SetClose(end).y,SetClose(end).x));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.计算船舶移动一步的距离代价movecost和应扩展的邻域节点数Num；
    movecost=10; %如果为变速的A*，所以movecost在这里，会改变
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For 生成的每一个邻域节点Surround(i)；
        Surround(ii).y=floor(SetClose(end).y-ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).x=floor(SetClose(end).x+ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).y,SetClose(end).x,Surround(ii).y,Surround(ii).x);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%不再计算相邻点的条件
        if ~isempty( SetOpen)
            openitear=1;
            mindis = 1000;
            while (openitear<length(SetOpen))
                dis=sqrt((Surround(ii).y -SetOpen(openitear).y)^2+(Surround(ii).x-SetOpen(openitear).x)^2);
                if(dis<mindis)
                    mindis=dis;
                    replace=openitear;
                end
                openitear=openitear+1;
            end
            if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).y,Surround(ii).x,SetOpen(replace).y,SetOpen(replace).x,ShipLong/2)==1)
                %                         if (mindis<6)
                Surround(ii).y=SetOpen(replace).y;
                Surround(ii).x=SetOpen(replace).x;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % line10.If(Sourround(i)在目标区域范围之内，且Sourround(i)处不是障碍物，且Sourround(i)不在close列表中,且从FMin移动到Sourround(i)过程中船舶安全,且运动过程不受船舶运动规律限制)
        if (0>=Surround(ii).x||Surround(ii).x>=length(background(1,:))||0>=Surround(ii).y||Surround(ii).y>=length(background(:,1))...
                || background(Surround(ii).y,Surround(ii).x)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).y,SetClose(end).x,Surround(ii).y,Surround(ii).x,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).y,Surround(ii).x,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. 计算Sourround(i)的G、H、F值,设置FMin为Sourround(i)的父节点；
            Surround(ii).H=sqrt((destination.x-Surround(ii).x)^2+(destination.y-Surround(ii).y)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).y,Surround(ii).x);%movecost用于调整势场代价值
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %设置FMin为Sourround(i)的父节点；
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)所在坐标不同于open列表中任意点坐标)
                SetOpen=[SetOpen;Surround(ii)]; %line13. 将Sourround(i)加入open列表；
            else %line14
                %% line15.比较Sourround(i)与open列表中具有相同坐标节点的G值，设置较小者的父节点为FMin；
                for kk=1:length(SetOpen)
                    %                         if abs(Surround(ii).y - SetOpen(kk).y)<=1/4*ShipLong && abs(Surround(ii).x-SetOpen(kk).x)<=1/4*ShipLong
                    if (Surround(ii).y == SetOpen(kk).y && Surround(ii).x==SetOpen(kk).x)
                        rember=kk;                       %找到Sourround(i)与open列表中具有相同坐标的节点
                    end
                end
                if Surround(ii).G < SetOpen(rember).G     %比较G值
                    SetOpen(rember).father=SetClose(end); %设置较小者的父节点为FMin；
                end
            end %line16.
        end     %line17.
    end         %line18.
    if SetClose(end).H < ShipSpeed %line19. 如果FMin到目标点的距离小于移动步长，算法结束；
        break;
    end
end
destination.father=SetClose(end);
destination.Dir=ShipDirection(SetClose(end).x,SetClose(end).y,end_x,end_y);
%绘制路径
figure
ss=pcolor(background);  %来自pcolor的官方示例
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
set(ss, 'LineStyle','none');
% rectangle('position',[1 1 size(background)-1],'edgecolor','k')%设置图片边框大小及颜色
t=1;
M(t)=getframe;
t=t+1;

CurrentPoint=destination;
PosTemp=[];
courseTemp=[];
posData0=[];
courseData=[];
while ~(CurrentPoint.x==start_point.x && CurrentPoint.y==start_point.y)
    position=[CurrentPoint.x  CurrentPoint.y];
    plotShip(fliplr(position),CurrentPoint.Dir,ShipLong/2);
%     ship_icon(position(1),position(2),Boat.State(4,5)/5, Boat.State(4,6)/5, CurrentPoint.Dir,1 );
    PosTemp=[PosTemp;position];
    courseTemp=[courseTemp;CurrentPoint.Dir];
    CurrentPoint=CurrentPoint.father;
    M(t)=getframe;
    t=t+1;
    
end
line([start_point.x-3;start_point.x+3;start_point.x+3;start_point.x-3;start_point.x-3],[start_point.y-3;start_point.y-3;start_point.y+3;start_point.y+3;start_point.y-3],'color','g','LineWidth',5);
line([destination.x-3;destination.x+3;destination.x+3;destination.x-3;destination.x-3],[destination.y-3;destination.y-3;destination.y+3;destination.y+3;destination.y-3],'color','b','LineWidth',5);

posData0=[posData0;flipud(PosTemp)];
deltaPos=300*ones(size(posData0));
posData=posData0-deltaPos;
courseData=[courseData;flipud(courseTemp)];
courseData_deg=180+courseData/pi*180;

figure
kk2=contourf(X,Y,APFValue);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
% set(kk2, 'LineStyle','none');
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),2 );

for i=1:1:length(posData)
    hold on;
    ship_icon(posData(i,1),posData(i,2),Boat.State(4,5)/5, Boat.State(4,6)/5, courseData_deg(i),1 );
end
% axis equal
% axis off
toc
disp(['本次运行时间: ',num2str(toc)]);
