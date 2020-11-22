%% 专门用于绘制ship1眼中的各类避碰场景
clear
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

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40
    ];

CAL0=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];

Boat_Num=4;   %船舶数量
Start_pos=[];
t=1;
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
end
%地图设置
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution地图的分辨率

[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%其他决策参数设置
for i=1:1:Boat_Num
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    Boat(i).FMPos=[];
    Boat(i).FMCourse=[];
    Boat(i).FMCourse_deg=[];
    Boat(i).Current_row=0;
    Boat(i).RiskHis=[];
    Boat(i).End=[];
    
end
tic
OS=1;

for scenario=0:1:0
    CAL_temp=dec2bin(scenario+8);
    CAL_temp2=CAL_temp(2:end);
    CAL_now=2;
    disp(['当前场景为',CAL_temp2]);
    for b=1:3
    CAL_now(b+1)=str2num(CAL_temp(b+1));
    end
    Boat(OS).CAL=CAL_now;
%% 建立本船的航行场
% 绘制当前本船眼中目标船的SCR
SCR_temp=zeros(m,n);
CAL_Field=zeros(m,n);
ScenarioMap=zeros(m,n);
AFMfield=zeros(m,n);
SCR=zeros(m,n);
PeakValue=100;
%0223 经过试验发现，在中间位置的时候，几艘船的航迹很诡异，4号船甚至开始震荡，
% 究其原因应该是在中间位置各个障碍物的各种场的叠加非常严重，导致无法正常决策
% 目前，加上碰撞风险DCPA的判断，如果没有碰撞风险，则不绘制势场图，只绘制本船的约束场
RiskLabel=[];
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        
        v_os = Boat(OS).speed(end,:);
        course_os = Boat(OS).COG_deg(end,:);
        pos_os = Boat(OS).pos(end,:);
        v_ts = Boat(TS).speed(end,:);
        course_ts = Boat(TS).COG_deg(end,:);
        pos_ts = Boat(TS).pos(end,:);
        
        k=k+1;
        
        Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
        Boat_Speed = Boat(TS).SOG(end,:);
        Shiplength = ShipSize(TS,1);
        
        SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
        
        %计算避碰规则下的风险场，规则场RuleField
        cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
        CAL=Boat(OS).CAL(TS);
        Rule_eta=2;
        Rule_alfa=0.1;
        %             CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
        CAL_Field= RuleField3(pos_ts(1),pos_ts(2),Boat_theta,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        SCR=SCR+SCR_temp;
    else
        RiskLabel(TS)=3;
    end
end
RiskLabel=[1,RiskLabel];
Boat(OS).RiskHis=[Boat(OS).RiskHis;RiskLabel];
RiskMap=1./(ScenarioMap+1);

% 绘制当前本船的航行遮罩
Boat_x=Boat(OS).pos(1,1);
Boat_y=Boat(OS).pos(1,2);
Boat_theta=-Boat(OS).COG_rad(end,:); %此处为弧度制
Shiplength = ShipSize(OS,1);
alpha=30;
R=500;
AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
GuidanceMap=1./(AFMfield+1);
safetymap=AFMfield+ScenarioMap;
SMap=1./(safetymap+1);
FM_map=10*min(RiskMap, GuidanceMap); % a, b是矩阵

%% 绘图测试
figure(scenario+1);
name=[CAL_temp2,'.jpg'];

kk1=mesh(X,Y,SCR);
% kk1=mesh(X,Y,ScenarioMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
draw=OS;
ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,0 )
axis equal

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0  300])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'FontSize',20);
xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',20);
ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',20);
zlabel('Risk Value', 'Fontname', 'Times New Roman','FontSize',20);
box on;
grid on;
% view([0,90]);
% 保存图像为jpg
% h = figure(scenario+1);
% 
% saveas(h,name);




% figure
% kk2=contourf(X,Y,ScenarioMap);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% hold on
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% draw=OS;
% ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
% title('当前场景为')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
end
% figure
% mesh(X,Y,GuidanceMap);
% title('导引图')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
%
% figure
% mesh(X,Y,AFMfield);
% title('导引图')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
