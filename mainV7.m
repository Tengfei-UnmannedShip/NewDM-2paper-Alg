%% (3.5版本)A*算法只计算未来5步或10步
% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.A*与APF结合的第三版，A*和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
%   (2.5版本)用WangNing的ShipDomain代替原APF，扩展到海里为单位的地图
% 2.(3.0版本)添加路径点
%   (3.5版本)A*算法只计算未来5步或10步：
%       步骤1:只计算10步，记录下运行的时间和结果，与完全计算的做对比，确认只计算10步的准确性和效率；
%       步骤2:加入路径点，然后第一次决策计算10步，然后在第一个for循环开始时更新状态，每5步（本程序中是5分钟，
%            t=60*5）重新算一次路径点，如果没有路径点了，就以终点为目标，接下来的过程中，如果再次出现路径点，
%            继续以路径点为目标；
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
for i=1:1:Boat_Num
    
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground，第一行为对地速度，单位节
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %第一行为对地速度，单位米／秒
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground，第一行为初始航向（deg，正北（Y正向）为0）
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground，第一行为初始航向（rad，正北（Y正向）为0）
    Boat(i).HisCOG=[Boat(i).COG_rad,Boat(i).COG_deg];
    %由中间位置倒推的初始位置，此处pos单位为米,随后每个时刻新增一行
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    Boat(i).HisPos=Boat(i).pos;
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    
    Boat(i).WayPoint_temp = [];
    Boat(i).WayPoint = [];
    Boat(i).HisWP = [];
    
    Boat(i).As_lable=0; %初始时刻As_lable为0，直到第一个时刻计算A*
    Boat(i).AsPos=[];
    Boat(i).AsCourse=[];
    Boat(i).AsCourse_deg=[];
    
end

for t=1:60:tMax
    t_count11=t_count12;    %时间计数
    
    %% 每个时刻的状态更新
    for i=1:1:Boat_Num
        if Boat(i).As_lable~=0
            Boat(i).As_lable = Boat(i).As_lable+1;
            Current_row=Boat(i).As_lable;
            Boat(i).pos = Boat(i).AsPos(Current_row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).AsCourse_deg(Current_row,:);
            Boat(i).COG_rad = Boat(i).AsCourse(Current_row,:);
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
        else
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
    end
    
    %% 路径点计算
    % =========================================================================
    % 输入：当前的每艘船的位置
    % 输出：每一艘本船眼中的每一艘目标船的路径点WP。
    % 基本步骤：
    % 1.计算每一艘船舶的船头船尾的两个waypoint(WP),编号分别为0(船头点)和1(船尾点);
    % 2.从任意船为主的角度，有8个场景(000-111)，从1000到1111，采用for循环产生十进制数再转换成2进制编号的方式生成8个场景
    % 3.8个场景中的每一个，都有3个点，根据公式，找到这3个点的中间点(按照3个点和到三艘船的距离的三元二次方程组)，这个中间点是真正的路径点WP
    % 注意：
    % 1.从谁的角度出发的问题，本船眼中的目标船和本船眼中的目标船眼中的其他船
    % =========================================================================
    
    for OS=1:1:Boat_Num
        k=1;
        for TS=1:1:Boat_Num
            if TS~=OS
                v_os = Boat(OS).speed(end,:);
                course_os = Boat(OS).COG_deg(end,:);
                pos_os = Boat(OS).pos(end,:);
                v_ts = Boat(TS).speed(end,:);
                course_ts = Boat(TS).COG_deg(end,:);
                pos_ts = Boat(TS).pos(end,:);
                TSlength = ShipSize(TS,1);
                d_thre = 1*1852;  %d_thre为判断碰撞风险的风险阈值
                changeLabel = 0;
                % =========================================================================
                % 先判断有碰撞风险，且需要计算waypoint的
                % 判断标准：是否有碰撞风险
                % 注意：可能出现原来试验中本来应该已经过去，但是预测路径汇聚在诡异的船头一点的情况
                % 当时推测是由于试验的数据不是A*算出来的数据的原因，这次再试一试，不要掉以轻心
                % =========================================================================
                CurrentRisk=CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre);
                if CurrentRisk==1  %即的确有碰撞风险
                    WP_label(k) = TS;
                    % 开始计算waypiont
                    Dis_temp(k) = norm(pos_os-pos_ts);
                    WayPoint_temp0 = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
                    WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0是船头点
                    WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1是船尾点
                    k=k+1;
                end
            end
        end
        WP_Num = length(WP_label);
        Boat(OS).curWP_label=WP_label;
        kk = 1;
        if ~isempty(Boat(OS).curWP_label)   %如果Boat(OS).curWP_label不为空，即存在有碰撞风险的船
            for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %有WP_Num艘风险船就是2^WP_Num个场景，共有WP_Num位
                %对每一个场景
                CAL_temp = dec2bin(scenario);
                for ts_i=1:1:WP_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
                    if     CAL_temp(ts_i+1)=='0'     %dec2bin函数输出的是字符，不是数字
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                    elseif CAL_temp(ts_i+1)=='1'
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP1;
                    end
                end
                switch WP_Num
                    case 1
                        WayPoint_OS(kk,:) = WayPoint_temp1;
                    case 2
                        WayPoint_OS(kk,:) = WP_3Ship(WayPoint_temp1(1,:),Dis_temp(1),WayPoint_temp1(2,:),Dis_temp(2));
                    case 3
                        WayPoint_OS(kk,:) = WP_4Ship(Dis_temp,WayPoint_temp1);
                end
                kk=kk+1;
            end
        end
        Boat(OS).WayPoint_temp = WayPoint_OS; %这样就得出了在t时刻所有场景的综合waypoint
    end
    %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
    for i=1:1:Boat_Num
        if shipLabel(i,2)==0    % inferLabbel:是否推测:0.不推测,1.推测;
            % 不推测，即不改变CAL，即按照最初的CAL，每艘船都知道彼此的CAL
            Boat(i).CAL=CAL0(i,:);
        elseif  shipLabel(i,2)==1
            % 贝叶斯推测
            
            Boat(i).CAL=CAL0(i,:); %贝叶斯推断最后得出的，还是当前时刻的Boat(i).CAL
        end
        
        CAL_temp1=Boat(i).CAL;
        WP_label1=Boat(i).curWP_label;
        scenario=CAL_temp1(WP_label1);
        WP_Num=length(Boat(i).curWP_label);
        
        switch WP_Num
            case 1        %只有1艘目标船时
                lable=scenario(1)+1;
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
            case 2        %有2艘目标船时
                lable=2*scenario(1)+scenario(2)+1;
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
            case 3        %有3艘目标船时
                lable=4*scenario(1)+2*scenario(2)+scenario(3)+1;
                disp(['当前为',num2str(i),'号船的场景 ',num2str(lable)]);
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
        end
        Boat(i).HisWP = [Boat(i).HisWP;Boat(i).WayPoint];
    end
    %% 绘制当前每艘船的SCR
    for i=1:1:Boat_Num
        
        Boat_x = Boat(i).pos(end,1);
        Boat_y = Boat(i).pos(end,2);
        Boat_theta = -Boat(i).COG_rad(end,:); %此处为弧度制
        Boat_Speed = Boat(i).SOG(end,:);
        Shiplength = ShipSize(i,1);
        
        Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
    end
    
    %% A*算法主程序
    for i=1:1:Boat_Num
        t_count21=t_count22;    %时间计数
        %       在当前时刻重新计算A*的条件包括：
        %          1.之前的A*计算结果已经用尽，需要重新计算；
        %          2.产生的新的路径点距离上一个路径点的相差2个格子以上；
        %          3.没有新的路径点，路径点变更为目标点
        if Boat(i).As_lable>=size(Boat(i).AsPos,1)||norm(Boat(i).HisWP(end-1,:)-Boat(i).WayPoint)>=2*Res ...
                ||isempty(Boat(i).curWP_label)
            RiskMap=zeros(m,n);
            for k=1:1:Boat_Num
                if k~=i
                    RiskMap=RiskMap+Boat(k).SCR;
                end
            end
            if  ~isempty(Boat(i).curWP_label)
                end_x = round((Boat(i).WayPoint(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).WayPoint(1,2)+MapSize(2)*1852)/Res);
            else
                end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);
            end
            start_x = round((Boat(i).pos(end,1)+MapSize(1)*1852)/Res);
            start_y = round((Boat(i).pos(end,2)+MapSize(2)*1852)/Res);
            start_theta = Boat(i).COG_rad(end,:);   %起始点艏向，弧度制
            %A*算法的目标点，在每个时刻，添加路径点后，应该首先是路径点，经过路径点后，才是这个最终的目标点
            
            ShipLong=2*round(ShipSize(i,1)/Res/2);     %船长先除以2取整再乘2是为了程序中用到ShipLong/2时,也可以保证为整数
            Movelength=round((Boat(i).speed(end,:)*60)/Res);  %步长,每分钟行进距离
            SurroundPointsNum=20; %跳整方向数，n向的A*
            valueAPF=2;  %APF势场的价值函数
            NodeOpti=0;
            step_num=2000;
            map=RiskMap;
            posData = [start_x,start_y];
            while size(posData,1)<=3
                step_num=2*step_num;
                [posData,courseData,courseData_deg] = Astar_step(step_num,map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize,Res);
            end
            %% 每次计算完局部A*之后，都会更新为最新的
            Boat(i).AsPos=posData;
            Boat(i).AsCourse=courseData;
            Boat(i).AsCourse_deg=courseData_deg;
            Boat(i).As_lable=1;     %每次重新计算，都将位置的标识位重新回归1，以后每次加1，知道用完当前的计算
            t_count22=toc;
            disp([num2str(i),'号船计算',num2str(step_num),'步的运行时间: ',num2str(t_count22-t_count21)]);
        end
    end
    t_count12=toc;    %时间计数
    disp([num2str(t),'时刻的运行时间: ',num2str(t_count12-t_count11)]);
end
t3=toc;
disp(['本次运行总时间: ',num2str(t3)]);
