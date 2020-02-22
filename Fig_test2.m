clear
clc
close all

tic;%tic1
%% 初始设置
% =========================================================================
% 船舶参数设置
% =========================================================================
% 1.compliance:避碰规则符合性:0.不遵守COLREGs,1.遵守COLREGs;
% 2.inferLabbel:是否推测:0.不推测,1.推测;
shipLabel=[
    1 0
    1 0
    1 0
    1 0];
%1~2位置(中间的位置，不是起始位置)、3航速(节)、4初始航向（deg，正北为0），5决策周期时长，6检测范围（range，nm）
%交叉相遇场景设置
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];
% % 追越场景设置
% ShipInfo=[
%     0.0,  0.0,  18,    0,  3,  6
%     0.0,  0.0,  13,    0,  4,  6
%     0.0,  0.0,  16,  300,  5,  6
%     0.0,  0.0,  13,  135,  5,  6
%     ];

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40
    ];
%初始场景，所有船舶全部正常
CAL0=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

Boat_Num=4;   %船舶数量
tMax=3600;    %最大时间
tBayes=2000;  %贝叶斯推测时间
t_count11=0;    %时间计数
t_count12=0;    %时间计数
t_count21=0;    %时间计数
t_count22=0;    %时间计数
Start_pos=[];
%地图设置
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %到达标志，到达目标点时为0，未到达时为1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground，第一行为对地速度，单位节
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %第一行为对地速度，单位米／秒
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground，第一行为初始航向（deg，正北（Y正向）为0）
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground，第一行为初始航向（rad，正北（Y正向）为0）
    Boat(i).HisCOG=[Boat(i).COG_rad,Boat(i).COG_deg];
    %由中间位置倒推的初始位置，此处pos单位为米,随后每个时刻新增一行
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    Start_pos = [Start_pos;Boat(i).pos];
    Boat(i).HisPos=Boat(i).pos;
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852);
end

% CAL0(1,:)=[2 0 1 1];
t_count11=t_count12;    %时间计数
reach_label=0; %每个时刻归零
%% 每个时刻的状态更新
for OS=1:1:1    %Boat_Num
    %判断当前i时刻是在OS船的决策周期中,compliance==1即本船正常,且未到达目标点
    
    %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
    if shipLabel(OS,2)==0    % inferLabbel:是否推测:0.不推测,1.推测;
        % 不推测，即不改变CAL，即按照最初的CAL，每艘船都知道彼此的CAL
        Boat(OS).CAL=CAL0(OS,:);
    elseif  shipLabel(OS,2)==1
        % 贝叶斯推测
        Boat(OS).CAL=CAL0(OS,:); %贝叶斯推断最后得出的，还是当前时刻的Boat(i).CAL
    end
    %% 本船路径规划主程序
    % 在当前时刻重新计算FM的条件包括：
    %   1.之前的FM计算结果已经用尽，需要重新计算；
    
    %% 建立本船的航行场
    % 绘制当前本船眼中目标船的SCR
    t_count31=toc;    %时间计数

    PeakValue=100;
    k=1;
    for TS=1:1:Boat_Num
        if TS~=OS
            SCR_temp{k}=zeros(m,n);
            CAL_Field{k}=zeros(m,n);
            ScenarioMap{k}=zeros(m,n);
            Boat_x = Boat(TS).pos(end,1);
            Boat_y = Boat(TS).pos(end,2);
            Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp{k} = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
            
            %计算避碰规则下的风险场，规则场RuleField
            CAL=Boat(OS).CAL(TS);
            CAL_Field{k} = RuleField( Boat_x,Boat_y,Boat_theta,Shiplength,MapSize,Res,PeakValue,CAL);
            ScenarioMap{k}=SCR_temp{k}+CAL_Field{k};
            k=k+1;

        end
    end
    Scenario=zeros(m,n);
    for i=1:1:k-1
        SCR_temp_max(i)=max((SCR_temp{i}(:)));
        CAL_Field_max(i)=max((CAL_Field{i}(:)));
        ScenarioMap_max(i)=max((ScenarioMap{i}(:)));
        
        Scenario=Scenario+ScenarioMap{i};
        
    end
    
    % 绘制当前本船的航行遮罩
    AFMfiled=zeros(m,n);
    Boat_x=Boat(OS).pos(1,1);
    Boat_y=Boat(OS).pos(1,2);
    Boat_theta=-Boat(OS).COG_rad(end,:); %此处为弧度制
    Shiplength = ShipSize(OS,1);
    alpha=30;
    AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,Shiplength,MapSize,Res,200);
    Scenario=AFMfiled+Scenario;
    
    %% 绘图测试
%     figure;
%     kk1=mesh(X,Y,Scenario);
%     colorpan=ColorPanSet(6);
%     colormap(colorpan);%定义色盘
%     hold on
%     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),2 );
%     axis equal
%     axis off
%     %     surf(X,Y,APFValue);
    
    figure
    % kk2=pcolor(APFValue);
    kk2=contourf(X,Y,Scenario);  %带填充颜色的等高线图
    colorpan=ColorPanSet(6);
    colormap(colorpan);%定义色盘
    % set(kk2, 'LineStyle','none');
    hold on
    plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
    hold on
%     ship_icon(Boat(OS).pos(1,1),Boat(OS).pos(1,2),ShipInfo(OS,5)*250,ShipInfo(OS,6)*50, Boat(OS).COG_deg(1),0);
    axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
    set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    grid on;
    xlabel('\it n miles', 'Fontname', 'Times New Roman');
    ylabel('\it n miles', 'Fontname', 'Times New Roman');
    box on;
    
end
