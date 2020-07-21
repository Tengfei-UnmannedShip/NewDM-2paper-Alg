%用于单船的路径规划测试
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
% 旧的逻辑01是反的
% CAL0=[
%     2 0 0 1
%     1 2 0 1
%     1 1 2 0
%     0 0 1 2];

CAL0=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];


Boat_Num=4;   %船舶数量
tMax=3600;    %最大时间
tBayes=2000;  %贝叶斯推测时间
t_count11=0;    %时间计数
t_count12=0;    %时间计数
t_count21=0;    %时间计数
t_count22=0;    %时间计数
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
Res=18.52*2;  %Resolution地图的分辨率

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
Boat(OS).CAL=CAL0(OS,:);

%% 建立本船的航行场
% 绘制当前本船眼中目标船的SCR
t_count31=toc;    %时间计数
SCR_temp=zeros(m,n);
CAL_Field=zeros(m,n);
ScenarioMap=zeros(m,n);
AFMfield=zeros(m,n);
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
        d_thre = 1*1852;                % d_thre为判断碰撞风险的风险阈值
        
        CPA_temp=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA_temp=CPA_temp(5);
        %如果DCPA_temp=0，后续会出现0所为被除数的事，因此最小的DCPA设置为1，如果太小的话，除完会很大，影响前面的参数
        if DCPA_temp<1
            DCPA_temp=1;
        end
        TCPA_temp=CPA_temp(6);
        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1即排除啊当前位置为CPA的情况
            CurrentRisk=1; %有碰撞风险为1
        else
            CurrentRisk=0;
        end
        Dis_temp=norm(pos_os-pos_ts);
        risklevel=0;
        %计算当前的目标船、有无风险、TCPA、DCPA、两船间的距离
        Risk_temp(k,:) = [TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp,risklevel];
        
        k=k+1;
        if  CurrentRisk==1 %有碰撞风险为1
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
            
            %计算避碰规则下的风险场，规则场RuleField
            
            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
            disp(['本船（',num2str(Boat(OS).COG_deg),'）与',num2str(OS),'号船（',num2str(Boat(TS).COG_deg),'）夹角为',num2str(cro_angle)]);
            
            CAL=Boat(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
            ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        end
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
figure;
kk1=mesh(X,Y,ScenarioMap);
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
axis equal
axis off

figure
kk2=contourf(X,Y,ScenarioMap);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
% set(kk2, 'LineStyle','none');
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
hold on
%                     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );


t_count32=toc;
disp([num2str(OS),'号船计算航行场用时: ',num2str(t_count32-t_count31)]);
%%寻找指引点

TCPA_OS=sum(Risk_temp(:,3));
DCPA_OS=sum(Risk_temp(:,4));
Dis_OS=sum(Risk_temp(:,5));
Risk_OS=Risk_temp;

Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
%这里的目的是找到风险最大的值，采用的方法是综合法，TCPA最紧迫，然后Dis，然后是DCPA，
%用100、10、1作为系数区分开，但是不知道效果如何，需要进一步的调试
Risk_OS(:,6)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
t_risk=t*ones(size(Risk_OS,1));


%% 目标船舶的路径点贝叶斯预测，确定真实的CAL
Risk_level=0;
Danger_TS=OS;
Risk_count=0;
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        if Risk_OS(k,2)~=0
            Risk_count=Risk_count+1;
            if Risk_OS(k,6)>Risk_level
                Risk_level=Risk_OS(k,6);
                Danger_TS=Risk_OS(k,1);
            end
        end
        k=k+1;
    end
    
end

%% FM算法主程序
start_point(1,2) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
start_point(1,1) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);


if  Danger_TS==OS   %即当前没有风险船，目标点为终点
    
    end_point(1,1) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
    end_point(1,2) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
    disp('当前没有风险船，目标点为终点');
    
else     %最危险的船即为当前的目标位置路径点
    disp(['当前风险船有',num2str(Risk_count),'艘，最危险的船为',num2str(Danger_TS)]);
    if   size(Boat(OS).End,1)>=1 && Boat(OS).End(end,2)==Danger_TS   %不是第一次决策且危险船与上次相同
        end_point= Boat(OS).End(end,3:4);
    else
        v_ts = Boat(Danger_TS).speed(end,:);
        course_ts = Boat(Danger_TS).COG_deg(end,:);
        pos_ts = Boat(Danger_TS).pos(end,:);
        TSlength = ShipSize(Danger_TS,1);
        changeLabel = 0; %不可变路径点
        
        WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel);
        %         WayPoint_temp = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
        if Boat(OS).CAL(Danger_TS)==0 %此时本船对该目标船是0，即本船为Stand-on直航船
            %此时本船应从目标船船头(fore section)过，即为船头的目标点
            WP= WayPoint_temp(1,1:2);
            disp('路径点为船头点');
        elseif Boat(OS).CAL(Danger_TS)==1  %此时本船对该目标船是1，即本船为Give-way让路船
            %此时本船应从目标船船尾(aft section)过，即为船尾的目标点
            WP= WayPoint_temp(1,3:4);
            disp('路径点为船尾点');
        end
        
        end_point(1,2) =round((WP(1,1)+MapSize(1)*1852)/Res);
        end_point(1,1) =round((WP(1,2)+MapSize(2)*1852)/Res);
        
    end
end

% end_point=[695,271];
Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];

% end_point(1,2)=round((-7.5*1852+MapSize(1)*1852)/Res);
% end_point(1,1)=round((0*1852+MapSize(2)*1852)/Res);


if size(Boat(OS).End,1)>1 && norm(Boat(OS).End(end-1,3:4)-end_point)<=2 ... %不是第一次运算的话，需要判断是否要维持上次的决策
        && Boat(OS).Current_row <= size(Boat(OS).path,1)-10         %当预决策的path只剩不到30个点时，强制重新计算
    disp('目标位置变动不大，则不计算');
else
    %当起始点或终点到边界处时，认为在内部
    if start_point(1,1)>m
        start_point(1,1)=m;
    end
    if start_point(1,2)>n
        start_point(1,2)=n;
    end
    if end_point(1,1)>m
        end_point(1,1)=m;
    end
    if end_point(1,2)>n
        end_point(1,2)=n;
    end
    
    t_count21=toc;

    [Mtotal, paths] = FMM(FM_map, end_point',start_point');
    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
    FinalMap=Mtotal;
    FMpath0 = paths{:};
    FMpath =FMpath0';
    path0=fliplr(FMpath);
    
    posData = zeros(size(path0));
    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
    end_point1(1)=end_point(2)*Res-MapSize(1)*1852;
    end_point1(2)=end_point(1)*Res-MapSize(2)*1852;
    Boat(OS).path=posData;
    Boat(OS).Current_row=1;   %每次重新决策，当前行数置1
    Boat(OS).decision_count=1;
    
end


% 绘图
% 绘图测试
figure
mesh(X,Y,FM_map);
title('当前输入地图')
figure
mesh(X,Y,RiskMap);
title('当前船舶安全图')
figure
mesh(X,Y,GuidanceMap);
title('当前导引图')
figure
mesh(X,Y,FinalMap)

figure
kk0=contourf(X,Y,FinalMap);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on

plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
hold on
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
hold on
plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-');
hold on

plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
hold on
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
hold on

plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
hold on
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
hold on

plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
hold on
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
hold on
plot(end_point1(1),end_point1(2),'r*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title('当前FM导出图')

box on;




figure
kk2=contourf(X,Y,safetymap);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on

plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
hold on
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
hold on
plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-');
hold on

plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
hold on
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
hold on

plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
hold on
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
hold on

plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
hold on
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
hold on
plot(end_point(1),end_point(2),'r*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;