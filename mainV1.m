%% A*与APF结合的第二版，找准当前位置与目标点
%%直接调用函数，% 原版作者薛双飞
clear all;
clc;

[X,Y]=meshgrid(-300:1:300,-300:1:300);  
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
Boat.State(4,:)=[ 120  -150   -45    10  20   5]; %ship4是本船
%Boat.State(5,:)=[-70   -15    90    10  20   5];
goal=[-280  250];%此时本船在（120，-150）角度-45
k_near=10;%计算目标点附近引力需要的增益系数
k_far=10; %计算引力门槛之后引力恒定时的增益系数，一般与k_near相同，否则在门槛处会出现引力场突变
Po_att=100;%引力门槛半径
k=0.3;%引力系数
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);  
% U_att=0.5*k_near*d_obs_APF.^2;%不加引力门槛的

% U_att=0.5*k_near*d_goal.^2.*(d_goal<=Po_att)+k_far*Po_att*d_goal.*(d_goal>Po_att);%加引力门槛的
% U_att=d_goal.*k ;  %引力为恒力的，引力场是距离的一次函数

for i=1:1:Boat_Num
    %局部坐标系转换
        
    %航标参数初始化
        Boat_x(i)=Boat.State(i,1);                  %第i个船x坐标
        Boat_y(i)=Boat.State(i,2);                  %第i个船y坐标
        Boat_theta(i)=-Boat.State(i,3)/180*pi;       %第i个船艏向角度  
        Boat_Speed(i)=Boat.State(i,4);              %第i个船速度大小
        Boat_length(i)=Boat.State(i,5);             %第i个船度
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

%% 绘图
figure
    mesh(X,Y,APFValue);
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );
    axis equal;
    axis off;
%     surf(X,Y,APFValue);
figure
    contourf(X,Y,APFValue,'LevelStep',30);  %带填充颜色的等高线图
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    hold on;
    ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );

%     plot(goal(1,1),goal(1,2),'rx');
    axis equal;
    axis off;

figure
%直接调用函数，% 原版作者薛双飞
[SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100); %函数定义CircleAStar(map,start_row,start_col,end_row,end_col)