%% (4.25版本)计算当前CAL下的完整过程，当前为追越
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%    最终版为FMmainV25.m，可以成功运行，但是由于没有航向角度的限制，导致规划出的航迹不可行
% 4.(5.0版本)使用AFM算法，即加上本船当前时刻不可达位置的遮罩
%   (5.1版本)加上AFM
%       步骤1：制作简单的直线的未来航行区域的遮罩
%       步骤2：根据OE论文中基本的线性NOMOTO算法，制作更精细的遮罩
%       步骤3：尝试根据NOMOTO算法，制作舵效与速度随舵角变化的遮罩
%   (5.2版本)将路径点改为规则场，加上不推测的CAL
%       步骤1：将TRB论文中的规则场应用进来，按照固定的CAL场航行
%       步骤2：本船的航行场，每艘船的风险场，每艘船眼中当前的CAL场的搭配与调节
% 5.(6.0版本)加上CAL推测，放大精度，蒙特卡洛方法计算多次的预测结果，并进行贝叶斯估计
%       步骤1：根据监测范围和风险船的数目确定场景数，根据不同的场景生成推测的TS眼中的场景，
%             并用推测的TS的航行方法进行蒙特卡洛仿真，为了保证速度，精度可以挑到200m或185.2m,甚至更大
%       步骤2：
%       步骤3：
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




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
Res=50;  %Resolution地图的分辨率
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
    
end

for t=1:1:2500
    t_count11=t_count12;    %时间计数
    reach_label=0; %每个时刻归零
    %% 每个时刻的状态更新
    for i=1:1:Boat_Num
        if Boat(i).FM_lable~=0 && Boat(i).reach==1 %即已经开始决策但没有到终点，此时按照决策来
            pos_temp=0;
            row=Boat(i).Current_row;
            while pos_temp<Boat(i).speed
                
                delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                delta_pos=norm(delta_pos0);
                pos_temp=pos_temp+delta_pos;
                row=row+1;
                
            end
            detaPos=Boat(i).path(row,:)-Boat(i).pos;
            Boat(i).pos = Boat(i).path(row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = NavAng(detaPos);
            Boat(i).COG_rad = Boat(i).COG_deg/180*pi;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            Boat(i).Current_row=row;
            
        elseif Boat(i).reach==0  %已经到达终点
            Boat(i).pos = Boat(i).pos;
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).COG_deg;
            Boat(i).COG_rad = Boat(i).COG_rad;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
        elseif Boat(i).FM_lable==0  %没有决策过的状态
            
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=2*Res %本船当前距离在同一个格子里，即认为本船到达目标点
            disp([num2str(t),'时刻',num2str(i),'号船到达目标点']);
            Boat(i).reach=0;
        end
        reach_label=reach_label+Boat(i).reach;
    end
    if reach_label<=1    %没到终点时为1，到了为0，因此，至少3艘船到达终点后reach_label<=1
        disp([num2str(t),'时刻','所有船完成避碰，计算结束']);
        break
    end
    
    for OS=1:1:1    %Boat_Num
        %判断当前i时刻是在OS船的决策周期中,compliance==1即本船正常,且未到达目标点
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(i).reach==1
            disp([num2str(t),'时刻',num2str(OS),'船开始决策']);
            
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
            if Boat(OS).FM_lable>=size(Boat(OS).FMPos,1)
                Calculate_lable=1;
                
            else
                Calculate_lable=0;
            end
            
            if Calculate_lable==1
                %% 建立本船的航行场
                % 绘制当前本船眼中目标船的SCR
                t_count31=toc;    %时间计数
                                        SCR_temp=zeros(m,n);
                        CAL_Field=zeros(m,n);
                        ScenarioMap=zeros(m,n);
                PeakValue=100;
                for TS=1:1:Boat_Num
                    if TS~=OS

                        Boat_x = Boat(TS).pos(end,1);
                        Boat_y = Boat(TS).pos(end,2);
                        Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(TS,1);
                        
                        SCR_temp= ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
                        
                        %计算避碰规则下的风险场，规则场RuleField
                        CAL=Boat(OS).CAL(TS);
                        CAL_Field= RuleField( Boat_x,Boat_y,Boat_theta,Shiplength,MapSize,Res,PeakValue,CAL);
                        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
                        
                    end
                end

                
                % 绘制当前本船的航行遮罩
                Boat_x=Boat(OS).pos(1,1);
                Boat_y=Boat(OS).pos(1,2);
                Boat_theta=-Boat(OS).COG_rad(end,:); %此处为弧度制
                Shiplength = ShipSize(OS,1);
                alpha=30;
                AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,Shiplength,MapSize,Res,200);
                ScenarioMap=ScenarioMap+AFMfiled;
                
                %     %% 绘图测试
                %     figure;
                %     kk1=mesh(X,Y,Scenario);
                %     colorpan=ColorPanSet(6);
                %     colormap(colorpan);%定义色盘
                %     hold on
                %     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %     hold on;
                %     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
                %     axis equal
                %     axis off
                %     %     surf(X,Y,APFValue);
                %
                %     figure
                %     % kk2=pcolor(APFValue);
                %     kk2=contourf(X,Y,Scenario);  %带填充颜色的等高线图
                %     colorpan=ColorPanSet(6);
                %     colormap(colorpan);%定义色盘
                %     % set(kk2, 'LineStyle','none');
                %     hold on
                %     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %     hold on
                %     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );
                %     % axis equal
                %     % axis off
                FM_map=1./ScenarioMap;
                t_count32=toc;
                disp([num2str(OS),'号船计算航行场用时: ',num2str(t_count32-t_count31)]);
                %% FM算法主程序
                start_point(1,1) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                start_point(1,2) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                
                end_point(1,1) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
                end_point(1,2) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
                
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
                [Mtotal, paths] = FMM(FM_map, end_point', start_point');
                Boat(OS).FM_lable=Boat(OS).FM_lable+1;
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
