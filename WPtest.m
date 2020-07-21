clear
clc
% close all

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
    0.0, 0.0,  20,    0,    3,  6
    0.0, 0.0,  20,  230,    4,  6
    0.0, 0.0,  18,  300,    5,  6
    0.0, 0.0,  17,  135,    5,  6
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
% 初始场景，所有船舶全部正常
% 关于CAL的规定，每次使用的都是一行，就是OS对TS的CAL，
% 例如CAL0(1,2)=0，意思是本船对TS的态度是0，即本船什么都不做，是stand-on ship，OS要从TS的船头过
CAL0=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];
% 根据初始状态的情况设置的风险感知参数，数值越小路径点距离该船越近，行为OS，列为TS
% 设置原则是按照初始状态，每艘船都倾向于优先避让本船右侧的船舶，因此右侧船系数为1
Risk_value=[5,1,10
    10,5,1
    10,1,5
    1,10,5];
 syms x y
 
Boat_Num=4;   %船舶数量
tMax=3600;    %最大时间
tBayes=2000;  %贝叶斯推测时间
t_count11=0;    %时间计数
t_count12=0;    %时间计数
t_count21=0;    %时间计数
t_count22=0;    %时间计数

%地图设置
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %到达标志，到达目标点时为0，未到达时为1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground，第一行为对地速度，单位节
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %第一行为对地速度，单位米／秒
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground，第一行为初始航向（deg，正北（Y正向）为0）
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground，第一行为初始航向（rad，正北（Y正向）为0）
    Boat(i).HisCOG=[];
    %由中间位置倒推的初始位置，此处pos单位为米,随后每个时刻新增一行
    pos0=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    %把初始位置归一到栅格位置上，但是依然有正负
    Boat(i).pos(1,1) = round(pos0(1,1)/Res)*Res;
    Boat(i).pos(1,2) = round(pos0(1,2)/Res)*Res;
    Boat(i).RiskHis = [];
    Boat(i).HisPos=[];
    %上一次决策的运算结果
    Boat(i).path = [];
    Boat(i).infercount=0;
end


%其他决策参数设置
for i=1:1:Boat_Num
    %把目标点位置归一到栅格位置上，但是依然有正负
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    Boat(i).waypoint=[];
    Boat(i).WP1=[];
    Boat(i).WP2=[];
    Boat(i).End=[];
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    Boat(i).decision_delay=0; %初始时刻decision_delay为0，直到第一个时刻计算FM
    Boat(i).Current_row=0;
    for ii=1:1:Boat_Num
        %infer_label是对目标船的推测次数的累计，格式为[上次推测的时刻，TS总的推测次数]
        Boat(i).InferData(ii).infer_label=[0,0];
    end
    Boat(i).Theta_his=[];
    %每艘船的CAL初设
    Boat(i).CAL=CAL0(i,:);       % Boat(OS).CAL是一个向量，表示OS对所有TS的CAL
    Boat(i).CAL_infer=CAL0(i,:); % Boat(OS).CAL_infer是一个向量，用于存储推测的CAL
end

%% 预计算，根据初始场景计算每艘船的所有waypoint，这写waypoint代表了不同的避碰场景，并在以后保持不变，直到CAL改变
for  OS=1:1:Boat_Num
    v_os          = Boat(OS).speed;
    course_os = Boat(OS).COG_deg;
    pos_os      = Boat(OS).pos;
    for TS=1:1:Boat_Num
        if TS~=OS
            v_ts          = Boat(TS).speed;
            course_ts = Boat(TS).COG_deg;
            pos_ts      = Boat(TS).pos;
            % 第一步，每艘船两个waypoint找齐
            WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,ShipSize(TS,1),0);
            %此时本船应从目标船船头(fore section)过，即为船头的目标点
            Boat(OS).WP1 = [Boat(OS).WP1; WayPoint_temp(1,1:2)];
            %此时本船应从目标船船尾(aft section)过，即为船尾的目标点
            Boat(OS).WP2 = [Boat(OS).WP2; WayPoint_temp(1,3:4)];
        end
    end
    WP_Num=Boat_Num-1;
    WayPoint_OS=[];
    for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %有WP_Num艘风险船就是2^WP_Num个场景，共有WP_Num位
        CAL_temp = dec2bin(scenario);
        for ts_i=1:1:WP_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
            CAL_ts=str2num(CAL_temp(ts_i+1));
            %此处以图4.1-1为标准，本船眼中，场景000的意思是，所有船都是对本船让路，本船从对方船头过，CAL场是大扇形
            if CAL_ts==0
                WayPoint_temp1(ts_i,:) = Boat(OS).WP1(ts_i,:);  %船头路径点
            elseif CAL_ts==1
                WayPoint_temp1(ts_i,:) = Boat(OS).WP2(ts_i,:);  %船尾路径点
            end
        end
        %找到当前场景中的waypoint，这是8个中的一个
        %         WayPoint_OS0=WP_4Ship(Risk_value(OS,:)',WayPoint_temp1);

        WayPoint_OS0 = ScenarioWaypoint( Risk_value(OS,:)',WayPoint_temp1);
        Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS0];
    end   %完成对当前OS所有8个路径点的收集
    
end

%识别初始场景
for  OS=1:4
    k=2;
    scen_os=1;
    for TS=1:4
        if TS~=OS
            scen_os=scen_os+CAL0(OS,TS)*(2^k);
            k=k-1;
        end
    end
    Boat(OS).currentWP=Boat(OS).waypoint(scen_os,:);
end

toc
figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end
for  scen=1:8
    plot(Boat(1).waypoint(scen,1),Boat(1).waypoint(scen,2),'r*');
end

for  scen=1:8
    plot(Boat(2).waypoint(scen,1),Boat(2).waypoint(scen,2),'b^');
end

for  scen=1:8
    plot(Boat(3).waypoint(scen,1),Boat(3).waypoint(scen,2),'gp');
end

for  scen=1:8
    plot(Boat(4).waypoint(scen,1),Boat(4).waypoint(scen,2),'kh');
end
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;

%画出当前场景

figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end

plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r*');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'b*');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'g*');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;