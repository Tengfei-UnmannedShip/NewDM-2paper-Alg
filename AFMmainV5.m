%% (5.5版本)CAL固定时的路径规划，最终成型
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 四艘船的实时航行，四艘船找到各自的路线，隔一段时间计算一次
% 0.(1.0版本)FM与APF结合的第三版，FM和APF都作为函数
%   0.1.临近节点优选作为函数的一个选项
% 1.(2.0版本)扩展到海里为单位的地图中
%   (2.5版本)用WangNing的ShipDomain代替原APF，扩展到海里为单位的地图
% 2.(3.0版本)添加路径点
%   (3.5版本)A*算法只计算未来5步或10步：
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
%       步骤3：直接在最开始就将实际坐标转换为栅格坐标，以后所有的运算绘图，都用栅格坐标，并且18.52为单位
%   (5.3版本)新的AFM计算方法：
%       经过和树武讨论，原来的AFM取值是没有用的，原因是生成方式有问题，解决方法：
%          (1)AFM场和安全场相结合取最小值生成新的场
%          (2)FM工具箱输入及输出的XY设置与我的正好相反，因此导致了很多诡异的路径。修改后再次试验。
%       具体步骤：
%        1.首先用旧的方法找到angle guidance的势场中所有不是0的点，找出所有位置
%        2.全是1的地图和上面的点的集合放入FMM函数，生成新的地图
%        3.注意，在angle guidance生成时的障碍物点列中xy是正的，但是在FMM路径规划中输入的起点终点和输出的路径xy都是反的
%   (5.4版本)解决了AFM中由于起点在安全地图的0值点无法计算的问题
%   (5.5版本)CAL固定时的路径规划，最终成型
%       步骤1：消除锯齿：重新决策时转换回栅格坐标则不转换了，直接从栅格坐标里找到对应项后以此作为起点。
%       步骤2：整合成函数：把个部分都整合成函数，主程序只提供时序
%       步骤3：
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
% 初始场景，所有船舶全部正常
% 关于CAL的规定，每次使用的都是一行，就是OS对TS的CAL，
% 例如CAL0(1,2)=0，意思是本船对TS的态度是0，即本船什么都不做，是stand-on ship，OS要从TS的船头过
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

%地图设置
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52*2;  %Resolution地图的分辨率
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
    pos0=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    %把初始位置归一到栅格位置上，但是依然有正负
    Boat(i).pos(1,1) = round(pos0(1,1)/Res)*Res;
    Boat(i).pos(1,2) = round(pos0(1,2)/Res)*Res;
    Boat(i).HisPos=[0,Boat(i).pos];
    %上一次决策的运算结果
    Boat(i).path = [];
    Boat(i).RiskData=[];
end


%其他决策参数设置
for i=1:1:Boat_Num
    %把目标点位置归一到栅格位置上，但是依然有正负
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    Boat(i).decision_count=0; %初始时刻decision_count为0，直到第一个时刻计算FM
    Boat(i).Current_row=0;
    Boat(i).RiskHis=[];
    Boat(i).End=[];         %记录危险船舶历史
    
end

for t=1:1:1100  %tMax*2
    t_count11=t_count12;    %时间计数
    reach_label=0; %每个时刻归零
    %% 每个时刻的状态更新
    for i=1:1:Boat_Num
        row=0;
        if Boat(i).FM_lable~=0 && Boat(i).reach==1 %即已经开始决策但没有到终点
            %由于FMM方法的特点，有些点之间步长特别长，因此需要分成两种情况
            row=Boat(i).Current_row;
            deltaPos=Boat(i).path(row+1,:)-Boat(i).pos;
            LastPos=Boat(i).pos;
            if  norm(deltaPos)>Boat(i).speed
                %步长大于单位时间船舶走的路程的时候，需要沿着规划之后的两点之间的线段找点
                disp([num2str(t),'时刻',num2str(i),'号船步长大于单位时间路程']);
                x_temp=Boat(i).pos(1);
                y_temp=Boat(i).pos(2);
                pos_temp0=[x_temp,y_temp];
                step_temp=0;
                while step_temp<Boat(i).speed
                    x_l=Boat(i).path(row:row+1,1);
                    y_l=Boat(i).path(row:row+1,2);
                    %采取插值函数找点
                    if x_l(1)==x_l(2)  %一条竖直线
                        x_temp=x_temp;
                        y_temp=y_temp+1;   %每次走1米
                    elseif y_l(1)==y_l(2)  %一条水平线
                        x_temp=x_temp+1;     %每次走1米
                        y_temp=y_temp;
                    else   %一条常规的斜线
                        x_temp=x_temp+1;     %每次x走1米
                        y_temp=interp1(x_l,y_l,x_temp,'spline');
                    end
                    pos_temp=[x_temp,y_temp];
                    delta_pos0=pos_temp-pos_temp0;
                    delta_pos=norm(delta_pos0);
                    step_temp=step_temp+delta_pos;
                    pos_temp0=pos_temp;
                end
                Boat(i).pos=pos_temp;
            else    %步长小于单位时间路程的时候，采用沿路径巡点的方法
                disp([num2str(t),'时刻',num2str(i),'号船步长小于单位时间路程']);
                row=row+1;
                step_temp=deltaPos;
                while step_temp<Boat(i).speed
                    delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                    delta_pos=norm(delta_pos0);
                    step_temp=step_temp+delta_pos;
                    row=row+1;
                end
                Boat(i).pos=Boat(i).path(row,:);
            end
            Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
            deltaPos=Boat(i).pos-LastPos;
            Boat(i).COG_deg = NavAng(deltaPos);
            Boat(i).COG_rad = Boat(i).COG_deg/180*pi;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            Boat(i).Current_row=row;
            
        elseif Boat(i).reach==0  %已经到达终点
            Boat(i).pos = Boat(i).pos;
            Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
            Boat(i).COG_deg = Boat(i).COG_deg;
            Boat(i).COG_rad = Boat(i).COG_rad;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
        elseif Boat(i).FM_lable==0  %没有决策过的状态
            
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=2*Res %本船当前距离在同一个格子里，即认为本船到达目标点
            disp([num2str(t),'时刻',num2str(i),'号船到达目标点']);
            Boat(i).reach=0;
        end
        reach_label=reach_label+Boat(i).reach;
    end
    if  Boat(1).reach==0
        disp([num2str(t),'时刻','1船完成避碰，计算结束']);
        break
    elseif Boat(2).reach==0
        disp([num2str(t),'时刻','2船完成避碰，计算结束']);
        break
    elseif Boat(3).reach==0
        disp([num2str(t),'时刻','3船完成避碰，计算结束']);
        break
    elseif Boat(4).reach==0
        disp([num2str(t),'时刻','4船完成避碰，计算结束']);
        break
    end
    if reach_label<=1    %没到终点时为1，到了为0，因此，至少3艘船到达终点后reach_label<=1
        disp([num2str(t),'时刻','所有船完成避碰，计算结束']);
        break
    end
    
    for OS=1:1:Boat_Num    %Boat_Num
        %判断当前i时刻是在OS船的决策周期中,compliance==1即本船正常,且未到达目标点
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(OS).reach==1
            
            if Boat(OS).decision_count<=round(20/ShipInfo(OS,5)) &&  Boat(OS).decision_count>0 ...    %每次决策之后保持20s
                    && Boat(OS).Current_row <= size(Boat(OS).path,1)-10         %当预决策的path只剩不到50个点时，强制重新计算
                
                disp([num2str(t),'时刻',num2str(OS),'船第',num2str(Boat(OS).decision_count),'周期保持上轮决策']);
                Boat(OS).decision_count=Boat(OS).decision_count+1;
                
            else
                disp([num2str(t),'时刻',num2str(OS),'船开始决策']);
                
                %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
                if shipLabel(OS,2)==0    % inferLabbel:是否推测:0.不推测,1.推测;
                    % 不推测，即不改变CAL，即按照最初的CAL，每艘船都知道彼此的CAL
                    Boat(OS).CAL=CAL0(OS,:);
                elseif  shipLabel(OS,2)==1
                    % 贝叶斯推测
                    Boat(OS).CAL=CAL0(OS,:); %贝叶斯推断最后得出的，还是当前时刻的Boat(i).CAL
                end
                
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
                        pos_os = Boat(OS).pos;
                        v_ts = Boat(TS).speed(end,:);
                        course_ts = Boat(TS).COG_deg(end,:);
                        pos_ts = Boat(TS).pos;
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
                            
                            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,50,2);
                            
                            %计算避碰规则下的风险场，规则场RuleField
                            
                            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
                            disp(['本船（',num2str(Boat(OS).COG_deg),'）与',num2str(TS),'号船（',num2str(Boat(TS).COG_deg),'）夹角为',num2str(cro_angle)]);
                            
                            CAL=Boat(OS).CAL(TS);
                            Rule_eta=2;
                            Rule_alfa=0.1;
                            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,30,CAL);
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
                alpha=30;    %30度在2*18.52的分辨率上太小了，在最后的AG_map上开口处会有一个诡异的尖刺，是由于开口附近的两个方格连在一起了
                R=500;
                AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                [AG_row,AG_col]=find(AFMfield~=0);
                AG_points=[AG_row,AG_col];
                AG_map0=ones(size(AFMfield));
                [AG_map, AGpaths] = FMM(AG_map0, AG_points');
                FM_map=min(AG_map,RiskMap)+0.0001;    %防止出现起始点或终点在0处，无法计算的情况
                
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
                if Boat(OS).FM_lable==0  %没有决策过
                    start_point(1,2) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                    start_point(1,1) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                else  %已经决策过，为了消除锯齿
                    row=Boat(OS).Current_row;
                    start_point=fliplr(Boat(OS).AFMpath(row,:));
                end
                
                if  Danger_TS==OS   %即当前没有风险船，目标点为终点
                    
                    end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
                    end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
                    disp('当前没有风险船，目标点为终点');
                    
                else     %最危险的船即为当前的目标位置路径点
                    disp(['当前风险船有',num2str(Risk_count),'艘，最危险的船为',num2str(Danger_TS)]);
                    if   size(Boat(OS).End,1)>=1 && Boat(OS).End(end,2)==Danger_TS   %不是第一次决策且危险船与上次相同
                        end_point= Boat(OS).End(end,3:4);
                    else
                        v_ts = Boat(Danger_TS).speed(end,:);
                        course_ts = Boat(Danger_TS).COG_deg(end,:);
                        pos_ts = Boat(Danger_TS).pos;
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
                
                Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];
                
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
                    
                    %     [Mtotal, paths] = FMM(FM_map, end_point',start_point');
                    [Mtotal, paths] = FMM(FM_map,start_point',end_point');
                    t_count22=toc;
                    disp([num2str(OS),'号船路径规划用时: ',num2str(t_count22-t_count21)]);
                    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
                    FMpath = paths{:};
                    path0=rot90(FMpath',2);
                    
                    posData = zeros(size(path0));
                    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                    
                    Boat(OS).AFMpath=path0;
                    if Boat(OS).pos(1)~=posData(1,1) && Boat(OS).pos(2)~=posData(1,2)
                        %如果当前位置不是决策的起点，则把当前位置放到起点
                        Boat(OS).path=[Boat(OS).pos;posData];
                    else
                        Boat(OS).path=posData;
                    end
                    Boat(OS).Current_row=1;   %每次重新决策，当前行数置1
                    Boat(OS).decision_count=1;
                    
                end
            end
        end
    end
    %
    %
    %     if t==1000
    %         Boat1000=Boat;
    %         t1000=t_count12;
    %         save('data0303-2000','Boat1000','t1000');
    %         disp('1000s数据已保存');
    %         clear Boat1000
    %     elseif t==1500
    %         Boat1500=Boat;
    %         t1500=t_count12;
    %         save('data0303-2000','Boat1500','t1500','-append');
    %         disp('1500s数据已保存');
    %         clear Boat1500
    %     elseif t==2000
    %         Boat2000=Boat;
    %         t2000=t_count12;
    %         save('data0303-2000','Boat2000','t2000','-append');
    %         disp('2000s数据已保存');
    %         clear Boat2000
    %     elseif t==2500
    %         Boat2500=Boat;
    %         t2500=t_count12;
    %         save('data0303-2000','Boat2500','t2500','-append');
    %         disp('2500s数据已保存');
    %         clear Boat2500
    %     end
    t_count12=toc;    %时间计数
    disp([num2str(t),'时刻的所有船舶的运行时间: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['本次运行总时间: ',num2str(t3)]);

Boat_end=Boat;
t_end=t_count12;
save('data0303-2000','Boat_end','t_end','-append');
disp('最终数据已保存');
clear Boat_end
