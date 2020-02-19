%% (4.2版本)计算当前CAL下的完整过程，当前为正常场景
% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.(1.0版本)FM与APF结合的第三版，FM和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
%   (2.5版本)用WangNing的ShipDomain代替原APF，扩展到海里为单位的地图
% 2.(3.0版本)添加路径点
%   (3.5版本)FM算法只计算未来5步或10步：
%       步骤1:只计算10步，记录下运行的时间和结果，与完全计算的做对比，确认只计算10步的准确性和效率；
%       步骤2:加入路径点，然后第一次决策计算10步，然后在第一个for循环开始时更新状态，每5步（本程序中是5分钟，
%            t=60*5）重新算一次路径点，如果没有路径点了，就以终点为目标，接下来的过程中，如果再次出现路径点，
%            继续以路径点为目标；
%   (3.7版本)采用单独路径点，不再采用综合路径点
%       步骤1:首先找到有风险的目标船，找到最有风险的那个，然后找到其相应的路径点
%       步骤2:一艘船一艘船的路过
% 3.(4.0版本)路径规划算法改用FM算法
%   (4.1版本)只计算1艘船的1步；
%       步骤1：目标点为路径点；
%       步骤2：目标点为路径点，识别已经到达路径点后，目标点改为终点
%   (4.2版本)计算当前CAL下的完整过程，记录时间；
%       步骤1：分清什么时候才用什么样的路径点，只要能够跑通，就是可以成功避碰的
%       步骤2：考虑CAL的作用？当前是固定CAL，在动态CAL下，会变成什么样呢？
% 4.(5.0版本)加入贝叶斯推断的内容



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
    Boat(i).WP_data = [];
    Boat(i).HisWP = [];
    
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    Boat(i).FMPos=[];
    Boat(i).FMCourse=[];
    Boat(i).FMCourse_deg=[];
    
end

for t=1:1:tMax
    t_count11=t_count12;    %时间计数
    
    %% 每个时刻的状态更新
    for i=1:1:Boat_Num
        if Boat(i).FM_lable~=0  %即已经开始决策了，此时按照决策来
            pos_temp=0;
            row=Boat(i).Current_row;
            while pos_temp>Boat(i).speed
                
                delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                delta_pos=norm(delta_pos0);
                pos_temp=pos_temp+delta_pos;
                row=row+1;
                
            end
            Boat(i).Current_row=row;
            Boat(i).pos = Boat(i).path(Current_row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).FMCourse_deg(Current_row,:);
            Boat(i).COG_rad = Boat(i).FMCourse(Current_row,:);
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
            
        else  %没有决策过的状态
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=Res %本船当前距离在同一个格子里，即认为本船到达目标点
            disp([num2str(t),'时刻',num2str(i),'号船到达目标点']);
            Boat(i).reach=0;
        end
    end
    if Boat(1).reach==0 && Boat(2).reach==0 && ...
            Boat(3).reach==0 && Boat(4).reach==0
        disp([num2str(t),'时刻','所有船到达目标点，计算结束']);
        break
    end
    
    for OS=1:1:Boat_Num
        
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 %判断当前i时刻是在j船的决策周期中且compliance==1即本船正常
            disp([num2str(t),'时刻',num2str(OS),'船开始决策']);
            t_count21=t_count22;    %时间计数
            %% 路径点计算
            % =========================================================================
            % 输入：当前的每艘船的位置
            % 输出：每一艘本船眼中的每一艘目标船的路径点WP。
            % 基本步骤：
            % 1.计算每一艘船舶的船头船尾的两个waypoint(WP),编号分别为0(船头点)和1(船尾点);
            % 2.首先找到所有风险的目标船，并找到最有风险的那个，然后找到其相应的路径点
            % =========================================================================
            WP_label=[];
            WayPoint_temp0=[];
            Risk_temp=[];
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
                        CPA_temp=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                        
                        %如果DCPA_temp=0，后续会出现0所为被除数的事，因此最小的DCPA设置为1，如果太小的话，除完会很大，影响前面的参数
                        DCPA_temp=CPA_temp(5);
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
                        
                        %计算当前的目标船、有无风险、TCPA、DCPA、两船间的距离
                        Risk_temp(k,:) = [TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp];
                        WayPoint_temp0 = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
                        WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0是船头点
                        WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1是船尾点
                        k=k+1;
                    end
                end
            end
            WP_Num = length(WP_label);
            TCPA_OS=sum(Risk_temp(:,3));
            DCPA_OS=sum(Risk_temp(:,4));
            Dis_OS=sum(Risk_temp(:,5));
            Risk_OS=Risk_temp;
            
            Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
            Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
            Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
            %这里的目的是找到风险最大的值，采用的方法是综合法，TCPA最紧迫，然后Dis，然后是DCPA，
            %用100、10、1作为系数区分开，但是不知道效果如何，需要进一步的调试
            Risk_value=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
            
            kk = 1;
            WayPoint_temp1 = [];
            for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %有WP_Num艘风险船就是2^WP_Num个场景，共有WP_Num位
                CAL_temp = dec2bin(scenario);
                for ts_i=1:1:WP_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
                    CAL_ts=str2num(CAL_temp(ts_i+1));
                    if CAL_ts==0
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                    elseif CAL_ts==1
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP1;
                    end
                end
                switch WP_Num
                    case 1
                        WayPoint_OS(kk,:) = WayPoint_temp1;
                    case 2
                        WayPoint_OS(kk,:) = WP_3Ship(WayPoint_temp1(1,:),Dis_temp(1),WayPoint_temp1(2,:),Dis_temp(2));
                    case 3
                        WayPoint_OS(kk,:) = WP_4Ship(Risk_value',WayPoint_temp1);
                end
                kk=kk+1;
            end
            [m_wp,n_wp] = size(WayPoint_OS);
            t_wp=t*ones(m_wp,1);
            Boat(OS).WP_data = [Boat(OS).WP_data;t_wp,WayPoint_OS]; %这样就得出了在t时刻所有场景的综合waypoint
            Boat(OS).WayPoint = WayPoint_OS;
            
            TCPA_OS=sum(Risk_temp(:,3));
            DCPA_OS=sum(Risk_temp(:,4));
            Dis_OS=sum(Risk_temp(:,5));
            Risk_OS=Risk_temp;
            
            Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
            Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
            Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
            %这里的目的是找到风险最大的值，采用的方法是综合法，TCPA最紧迫，然后Dis，然后是DCPA，
            %用100、10、1作为系数区分开，但是不知道效果如何，需要进一步的调试
            Risk_OS(:,2)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
            Boat(OS).Risk=Risk_OS;
            Boat(OS).WayPoint_temp = WayPoint; %这样就得出了在t时刻所有场景的综合waypoint
            
            %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
            
            if shipLabel(OS,2)==0    % inferLabbel:是否推测:0.不推测,1.推测;
                % 不推测，即不改变CAL，即按照最初的CAL，每艘船都知道彼此的CAL
                Boat(OS).CAL=CAL0(OS,:);
            elseif  shipLabel(OS,2)==1
                % 贝叶斯推测
                
                Boat(OS).CAL=CAL0(OS,:); %贝叶斯推断最后得出的，还是当前时刻的Boat(i).CAL
            end
            WayPoint_temp=[];
            Risk_level=0;
            k=1;
            for TS=1:1:Boat_Num
                if TS~=OS
                    if Boat(OS).Risk(k,2)~=0
                        if Boat(OS).Risk(k,2)>Risk_level
                            Risk_level=Boat(OS).Risk(k,2);
                            if Boat(OS).CAL(TS)==0
                                WayPoint_temp=Boat(OS).WayPoint_temp(k).WP0;
                            elseif Boat(OS).CAL(TS)==1
                                WayPoint_temp=Boat(OS).WayPoint_temp(k).WP1;
                            end
                        end
                    end
                else
                    continue
                end
                k=k+1;
            end
            if isempty(WayPoint_temp)
                Boat(OS).WayPoint=Boat(OS).goal;  %没有路径点的时候，直接用终点作为目标点
            else
                Boat(OS).WayPoint=WayPoint_temp;
            end
            Boat(OS).HisWP = [Boat(OS).HisWP;t,Boat(OS).WayPoint];
            
            %% 本船路径规划主程序
            % 在当前时刻重新计算FM的条件包括：
            %   1.之前的FM计算结果已经用尽，需要重新计算；
            %   2.产生的新的路径点距离上一个路径点的相差2个格子以上；
            %   3.没有新的路径点，路径点变更为目标点
            if size(Boat(OS).HisWP,1)>=2   %即不是第一个路径点
                if  norm(Boat(OS).HisWP(end-1,2:3)-Boat(OS).WayPoint)>=2*Res
                    Calculate_lable=1;
                end
            elseif Boat(OS).FM_lable>=size(Boat(OS).FMPos,1)
                Calculate_lable=1;
            elseif isempty(Boat(OS).curWP_label)
                Calculate_lable=1;
            else
                Calculate_lable=0;
            end
            
            if Calculate_lable==1
                %% 绘制当前本船的SCR
                RiskMap=ones(m,n);
                SCR_temp=zeros(m,n);
                
                for TS=1:1:Boat_Num
                    if TS~=OS
                        Boat_x = Boat(TS).pos(end,1);
                        Boat_y = Boat(TS).pos(end,2);
                        Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(OS,1);
                        
                        SCR_temp = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
                        RiskMap=RiskMap+SCR_temp;
                    end
                end
                FM_map=1./RiskMap;
                
                %% FM算法主程序
                start_point(1,1)  = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                start_point(1,2)  = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                
                end_point =round((Boat(OS).WayPoint+MapSize(1)*1852)/Res);
                t_count21=toc;
                [Mtotal, paths] = FMM(FM_map, end_point', start_point');
                
                path0 = paths{:};
                path0 =path0';
                
                posData = zeros(size(path0));
                posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                Boat(OS).path=posData;
                Boat(OS).Current_row=1;   %每次重新决策，当前行数置1
                t_count22=toc;
                disp([num2str(OS),'号船计算的运行时间: ',num2str(t_count22-t_count21)]);
            end
        end
    end
    t_count12=toc;    %时间计数
    disp([num2str(t),'时刻的所有船舶的运行时间: ',num2str(t_count12-t_count11)]);
end
t3=toc;
disp(['本次运行总时间: ',num2str(t3)]);
