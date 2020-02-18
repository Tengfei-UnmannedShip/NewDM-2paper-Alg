clear
tic;%tic1
%% 初始状态设置
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
GoalRange=MapSize-[0.5,0.5];
Res=10;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%其他决策参数设置
for i=1:1:Boat_Num
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    
    Boat(i).WayPoint_temp = [];
    Boat(i).WayPoint = [];
    Boat(i).HisWP = [];
    
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    Boat(i).FMPos=[];
    Boat(i).FMCourse=[];
    Boat(i).FMCourse_deg=[];
    
end


%% 势场计算
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).pos(end,1);
    Boat_y = Boat(i).pos(end,2);
    Boat_theta = -Boat(i).COG_rad(end,:); %此处为弧度制
    Boat_Speed = Boat(i).SOG(end,:);
    Shiplength = ShipSize(i,1);
    
    Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
end


%% 路径规划

for i=1:1:1
    RiskMap=zeros(m,n);
    for k=1:1:Boat_Num
        if k~=i
            RiskMap=RiskMap+Boat(k).SCR;
        end
    end
    RiskMap=ones(size(RiskMap))+RiskMap;
    FM_map=1./RiskMap;
    M = FM_map;
    
    start_point(1,1)  = round((Boat(i).pos(1,1)+MapSize(1)*1852)/Res);
    start_point(1,2)  = round((Boat(i).pos(1,2)+MapSize(2)*1852)/Res);
    
    end_point =round((Boat(i).goal+MapSize(1)*1852)/Res);
     t_count21=toc;
    [Mtotal, paths] = FMM(M, end_point', start_point');

    path0 = paths{:};
    
    path0 =path0';
    Boat(i).FMpath=path0;
    posData = zeros(size(path0));
    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    Boat(i).path=posData;
     t_count22=toc;
     disp([num2str(i),'号船计算的运行时间: ',num2str(t_count22-t_count21)]);
end
 t_count1=toc;
disp(['总运行时间: ',num2str(t_count1)]);



