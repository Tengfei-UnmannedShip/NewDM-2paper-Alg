%% (3.0版本)添加路径点
% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.A*与APF结合的第三版，A*和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
%   (2.5版本)用WangNing的ShipDomain代替原APF，扩展到海里为单位的地图
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

% =========================================================================
% 船舶参数设置
% =========================================================================
% 1.compliance:避碰规则符合性;2.inferLabbel:是否推测
shipLabel=[
    0 0
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
CAL=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

Boat_Num=4;   %船舶数量
tMax=2500;    %最大时间
tBayes=2000;  %贝叶斯推测时间

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
    Boat(OS).WP_data=[];
end

for t=1:1:tMax
    %% 每个时刻的状态更新
          
    
    
    
    
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
        kk = 1;
        for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %有WP_Num艘风险船就是2^WP_Num个场景，共有WP_Num位
            CAL_temp = dec2bin(scenario);
            for ts_i=1:1:WP_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
                if CAL_temp(ts_i)==0
                    WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                elseif CAL_temp(ts_i)==1
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
        [m_wp,n_wp] = size(WayPoint_OS);
        t_wp=t*ones(m_wp,1);
        Boat(OS).WP_data = [Boat(OS).WayPoint;t_wp,WayPoint_OS]; %这样就得出了在t时刻所有场景的综合waypoint
        Boat(OS).WayPoint = WayPoint_OS;
    end
    %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
    
    
    
    
    %% 绘制当前每艘船的SCR
    for i=1:1:Boat_Num
        
        Boat_x = Boat(i).pos(end,1);
        Boat_y = Boat(i).pos(end,2);
        Boat_theta = -Boat(i).COG_rad(end,:); %此处为弧度制
        Boat_Speed = Boat(i).SOG(end,:);
        Shiplength = ShipSize(i,1);
        
        Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
        
    end
    
    %% APF绘图测试程序
    % =========================================================================
    % SCR=zeros(m,n);
    % for i=1:1:Boat_Num
    %     SCR=SCR+Boat(i).SCR;
    % end
    %
    % % figure
    % % kk1=mesh(X,Y,SCR);
    % % colorpan=ColorPanSet(6);
    % % colormap(colorpan);%定义色盘
    % % axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0 150])
    % % set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % % set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % % set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % % set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % % grid on;
    % % xlabel('\it n miles', 'Fontname', 'Times New Roman');
    % % ylabel('\it n miles', 'Fontname', 'Times New Roman');
    % % title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
    % % box off;
    %
    % figure
    % [M,c] = contourf(X,Y,SCR);
    % c.LineWidth = 0.001;
    % % c.colormap=colorpan;%定义色盘
    % colorpan=ColorPanSet(6);
    % colormap(colorpan);%定义色盘
    %
    % for i=1:1:Boat_Num
    %     hold on
    %     ship_icon(Boat(i).pos(1,1),Boat(i).pos(1,2),ShipSize(i,1),ShipSize(i,2),Boat(i).COG_deg,0);
    % end
    % % axis([xmin xmax ymin ymax zmin zmax])
    % axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
    % set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % grid on;
    % xlabel('\it n miles', 'Fontname', 'Times New Roman');
    % ylabel('\it n miles', 'Fontname', 'Times New Roman');
    % title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
    % box on;
    % =========================================================================
    
    %% A*算法主程序
    for i=2:1:2
        start_x = round((Boat(i).pos(end,1)+MapSize(1)*1852)/Res);
        start_y = round((Boat(i).pos(end,2)+MapSize(2)*1852)/Res);
        start_theta = Boat(i).COG_rad(end,:);   %起始点艏向，弧度制
        %A*算法的目标点，在每个时刻，添加路径点后，应该首先是路径点，经过路径点后，才是这个最终的目标点
        end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
        end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);
        
        RiskMap=zeros(m,n);
        for k=1:1:Boat_Num
            if k~=i
                RiskMap=RiskMap+Boat(k).SCR;
            end
        end
        
        ShipLong=2*round(ShipSize(i,1)/Res/2);     %船长先除以2取整再乘2是为了程序中用到ShipLong/2时,也可以保证为整数
        Movelength=round((Boat(i).speed(end,:)*60)/Res);  %步长,每分钟行进距离
        SurroundPointsNum=20; %跳整方向数，n向的A*
        valueAPF=2;  %APF势场的价值函数
        NodeOpti=0;
        [posData,courseData,courseData_deg] = AstarMain(RiskMap,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
        
        Boat(i).AsPos=posData;
        Boat(i).AsCourse=courseData;
        Boat(i).AsCourse_deg=courseData_deg;
        for ii=1:1:length(Boat(i).AsPos)
            
            Boat(i).DecPos(ii,1)=Boat(i).AsPos(ii,1)*Res-MapSize(1)*1852;
            Boat(i).DecPos(ii,2)=Boat(i).AsPos(ii,2)*Res-MapSize(2)*1852;
            
        end
    end
    
    % %% A*绘制路径
    % =========================================================================
    % SCR=zeros(m,n);
    % for i=1:1:Boat_Num
    %     SCR=SCR+Boat(i).SCR;
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
    % =========================================================================
end
toc
disp(['本次运行时间: ',num2str(toc)]);
