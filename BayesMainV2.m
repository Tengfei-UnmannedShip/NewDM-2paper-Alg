%% (6.1版本)加上贝叶斯推测
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
% 5.(6.0版本)加上CAL推测，蒙特卡洛方法计算多次的贝叶斯估计和结果预测
%     总体思路：公式(1)更新当前位置分布，公式(2)更新当前的意图分布，根据意图概率生成围绕每一个路径点的点云
%             每一个点云的位置作为终点，终点坐标输入FM，生成n条路径，每一个路径回归到点，
%             通过计算在OS船头船尾的点的多少来确定CAL
%     步骤1.公式(1)更新当前位置分布
%     步骤2.公式(2)更新当前的意图分布
%     步骤3.根据意图概率，用MC方法生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
%     步骤4.每一个点云的位置作为终点，终点坐标输入FM，生成n条路径
%     步骤5.每一个路径回归到点，每一个路径点计数
%     步骤6.通过计算在OS船头船尾的点的多少来确定CAL
% (6.1版本)调试Bug
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    1 1
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
    Boat(i).infertime=0;
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
    %     Boat(i).End=[];         %记录危险船舶历史
    Boat(i).infer_label=zeros(1,Boat_Num);
    Boat(i).Theta_his=[];
end


% 正式决策开始
for t=1:1:6    %tMax*2
    t_count11=t_count12;    %时间计数
    Boat0=Boat;
    % 每时刻状态更新，在程序中运行的都只是必要的信息
    Boat=StateUpdate(Boat0,Boat_Num,t,Res);
    clear Boat0
    reach_label=0; %每个时刻归零
    reach_label=reach_label+Boat(1).reach+Boat(2).reach+Boat(3).reach+Boat(4).reach;
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
    % 判断避碰是否结束，没到终点时为1，到了为0，因此，至少3艘船到达终点后reach_label<=1
    % 如果用DCPA判断，有可能暂时的DCPA显示没有风险，但是后续还是会继续出现风险
    if reach_label<=1    
        disp([num2str(t),'时刻','所有船完成避碰，计算结束']);
        break
    end
    
    for OS=1:1:Boat_Num    %Boat_Num
        %TODO 各种判断条件的简化，感觉
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
                    Boat(OS).infertime=Boat(OS).infertime+1;
                    disp([' ',num2str(OS),'船第',num2str(Boat(OS).infertime),'次推测']);
                    %% 开始贝叶斯推测--测试通过后变成函数
                    % 推测的步骤
                    % 步骤1.公式(1)更新当前位置分布
                    % 步骤2.公式(2)更新当前的意图分布
                    % 步骤3.根据意图概率生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
                    % 步骤4.每一个点云的位置作为终点，终点坐标输入FM，生成n条路径
                    % 步骤5.每一个路径回归到点，每一个路径点计数
                    for  TS=1:1:Boat_Num
                        v_os      = Boat(OS).speed;
                        course_os = Boat(OS).COG_deg;
                        pos_os    = Boat(OS).pos;
                        v_ts      = Boat(TS).speed;
                        course_ts = Boat(TS).COG_deg;
                        pos_ts    = Boat(TS).pos;
                        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                        DCPA_temp=CPA_temp(5);TCPA_temp=CPA_temp(6);
                        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1即排除啊当前位置为CPA的情况
                            CurrentRisk=1; %有碰撞风险为1
                        else
                            CurrentRisk=0;
                        end
                        if TS~=OS && CurrentRisk==1   %真正有风险才推测，否则不推测
                            disp(['  进入',num2str(TS),'船视角']);
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % 正式开始TS视角的推测，这时TS就成了OS眼中的OS，一切从TS出发
                            % 输入：当前可以观测到的各种状态量
                            % 输出：推测的TS状态
                            %      1.TS对其他船的CAL??给出0/1的CAL向量和概率值，形式如下：
                            %        TS的CAL历史，详细记录每一次的CAL的预测结果
                            %        CalHis_TS=[t,TS,ts_infer,0,Pr(CAL=0);
                            %                   t,TS,ts_infer,0,Pr(CAL=0) ];
                            %        TS的CAL推测结果，取概率最大的值，没有碰撞风险的船取初始值补齐，例如：
                            %        CAL_TS=[0,1,1,0]
                            %      2.OS推测的TS的可能路径，是一张地图PreMap_TS，每一点是TS可能经过的位置的计数
                            %      3.最终，给OS的返回值：
                            %        Boat(OS).Infer(TS).InferHis=CalHis_TS%累积
                            %        Boat(OS).Infer(TS).CAL=[t,CAL_TS]%累积
                            %        Boat(OS).Infer(TS).PreMap=PreMap_TS%每次更新
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            
                            ScenarioMap=zeros(m,n);
                            v_os      = Boat(TS).speed;
                            course_os = Boat(TS).COG_deg;
                            pos_os    = Boat(TS).pos;
                            
                            for ts_infer=1:1:Boat_Num
                                
                                v_ts      = Boat(ts_infer).speed;
                                course_ts = Boat(ts_infer).COG_deg;
                                pos_ts    = Boat(ts_infer).pos;
                                CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                                if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1即排除啊当前位置为CPA的情况
                                    CurrentRisk_TS=1; %有碰撞风险为1
                                else
                                    CurrentRisk_TS=0;
                                end
                                if ts_infer~=TS && CurrentRisk_TS==1 %TS视角下，ts_infer(包括本船OS)和TS的确有碰撞风险
                                    % 步骤0.1.2. TS视角下所有可能的路径点
                                    % 成对出现，作为路径点的theta。
                                    % 设定：一艘船有可能有风险不规避，即为失控船，但是不可能没有风险还无谓的规避。
                                    %      一旦目标船没有风险船了，目标就只有自身的本船目标一点
                                    changeLabel = 0; %不可变路径点
                                    Shiplength = ShipSize(ts_infer,1);
                                    WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,Shiplength,changeLabel);
                                    %此时本船应从目标船船头(fore section)过，即为船头的目标点
                                    WP1 = WayPoint_temp(1,1:2);
                                    %此时本船应从目标船船尾(aft section)过，即为船尾的目标点
                                    WP2 = WayPoint_temp(1,3:4);
                                    if  Boat(OS).infer_label(TS)==0   %TS之前没有推测过，这是第一次
                                        % 步骤0.1. 找到所有theta
                                        % 当不是初始状态时，则直接提取沿用上个时刻的theta，只推测和预测，没有本步
                                        % 步骤0.1.1. 第一行是TS的目标点，如果TS不决策，也就是直接向着这一点去
                                        ThetaList0=[t,TS,TS,Boat(TS).goal,1,Boat(OS).infer_label(TS)];
                                        %第一次推测，因此使用预设的PrTheta
                                        if CAL0(TS,ts_infer)==0
                                            Pr1=0.8;   %从WP1过的可能性大
                                            Pr2=0.2;
                                        elseif CAL0(TS,ts_infer)==1
                                            Pr1=0.2;
                                            Pr2=0.8;   %从WP2过的可能性大
                                        end
                                        
                                        ThetaList0  =[ThetaList0;
                                            t,TS,ts_infer,WP1,Pr1,Boat(OS).infer_label(TS);
                                            t,TS,ts_infer,WP2,Pr2,Boat(OS).infer_label(TS)];
                                        
                                    else    %已经有了预测的记录后，采用上一个时刻的PrTheta
                                        ThetaList0=[t,TS,TS,Boat(TS).goal,1,Boat(OS).infer_label(TS)];
                                        % 从历史记录中提取出新的ThetaList0，并且只有在当前对TS有碰撞风险的ts_infer才计入ThetaList0
                                        WP_num=1;
                                        for k_inf=1:1:size(Boat(OS).Theta_his,1)
                                            if Boat(OS).Theta_his(k_inf,2)==TS && ...
                                                    Boat(OS).Theta_his(k_inf,3)==ts_infer && ...
                                                    Boat(OS).Theta_his(k_inf,7)==Boat(OS).infer_label(TS)
                                                ThetaList0  =[ThetaList0;
                                                    Boat(OS).Theta_his(k_inf,:)];
                                                %因为在ThetaList0的排序中，只要有WP，那么WP1一定排在WP2之前，所以，第一查到的就是WP1
                                                if WP_num==1
                                                    Pr1=ThetaList0(k_inf,6);
                                                else
                                                    Pr2=ThetaList0(k_inf,6);
                                                end
                                                WP_num=WP_num+1;
                                            end
                                        end
                                        %提取出的历史ThetaList0再归入Pr1，Pr2
                                    end
                                    % 步骤0.2. 绘制当前TS眼中的风险场、规则场和引导场
                                    % 计算避碰规则下的风险场，规则场RuleField
                                    Boat_theta = -Boat(ts_infer).COG_rad; %此处为弧度制
                                    Boat_Speed = Boat(ts_infer).SOG;
                                    % 风险场
                                    SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                                    cro_angle=abs(Boat(TS).COG_deg-Boat(ts_infer).COG_deg);
                                    CAL=CAL0(TS,ts_infer);
                                    Rule_eta=2;
                                    Rule_alfa=0.1;
                                    % 规则场
                                    CAL_Field0=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,0);
                                    CAL_Field1=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,1);
                                    % 规则场是两种状态按照概率的叠加，如果状态为0.5/0.5，则生成的路径不具有参考意义
                                    CAL_Field=Pr1*CAL_Field0+Pr2*CAL_Field1;
                                    % 总的环境场
                                    ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
                                end
                            end
                            RiskMap=1./(ScenarioMap+1);
                            %完成对TS所有可能theta的收集（步骤0.1.1完成）
                            Theta=ThetaList0(:,4:5); %只有当前所有的theta点
                            Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res)+1;
                            Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res)+1;
                            disp(['  完成对TS所有',num2str(size(Theta,1)),'个可能theta的收集']);
                            
                            % 当前TS的引导场
                            Boat_x=Boat(TS).pos(1,1);
                            Boat_y=Boat(TS).pos(1,2);
                            Boat_theta=-Boat(TS).COG_rad; %此处为弧度制
                            alpha=30;
                            R=500;
                            
                            AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                            [AG_row,AG_col]=find(AFMfield~=0);
                            AG_points=[AG_row,AG_col];
                            
                            AG_map0=ones(size(AFMfield));
                            [AG_map, ~] = FMM(AG_map0, AG_points');
                            FM_map=min(AG_map,RiskMap);
                            % TS的FMM初始位置
                            start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
                            start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
                            step_length=[Res,Res];
                            while  FM_map(start_point(1),start_point(2))<0.001
                                start_temp = ang_point(Boat_x,Boat_y,Boat(OS).COG_deg,step_length);
                                start_point(1,2) = round((start_temp(1)+MapSize(1)*1852)/Res)+1;
                                start_point(1,1) = round((start_temp(2)+MapSize(2)*1852)/Res)+1;
                                step_length=step_length+step_length;
                            end
                            [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
                            % 注意：得出的InferMap和L0_paths的都是先验的
                            % 1.InferMap是FMM方法得出的以当前TS位置为起点的上一个时刻的CAL概率为先验概率的地图
                            % 2.L0_paths是当前的所有theta得出的初步的路径
                            %% 步骤1.公式(1)更新当前位置分布，给下个时刻用
                            disp('  开始Equ1的计算');
                            t_eq11=toc;
                            [RR_points,PrX_eq1] = BayesEqu1(Boat_x,Boat_y,L0_paths,Boat_theta,Boat(TS).speed,Theta,Theta_end,FM_map,MapSize,Res);
                            % 每次更新可达点和对应的值，但是Theta不一样
                            t_eq12=toc;
                            disp(['  完成Equ1的计算，更新当前的位置分布,用时',num2str(t_eq12-t_eq11)]);
                            DisPrX=[RR_points,PrX_eq1]
                            Boat(OS).infer_points(TS)=RR_points;
                            Boat(OS).PrX(TS)=PrX_eq1;
                            %% 步骤2.公式(2)更新当前的意图分布
                            % 来自树武程序
                            PrTheta0=ThetaList0(:,6)/sum(ThetaList0(:,6)); %PrTheta归一化，防止出现越来越小的情况
                            PrTheta0=PrTheta0';        %第一次预测，第一行是初始的针对每一个theta的先验概率
                            if  Boat(OS).infer_label(TS)==1   %TS之前没有推测过，这是第一次
                                % 这里需要加上if语句
                                % 第一次预测，需要找到当前点在上一步的先验概率
                                % 此时的输入为TS上一步的状态
                                Boat_x=Boat(TS).HisPos(end-1,1);
                                Boat_y=Boat(TS).HisPos(end-1,2);
                                Boat_theta=-Boat(TS).HisCOG(end-1,1);
                                [PrRR_points,PrePrX_eq1] = BayesEqu1(Boat_x,Boat_y,Boat_theta,Boat(TS).speed,Theta,MapSize,Res);
                            else
                                % 之前计算过，直接采用上一步的结果
                                PrRR_points=Boat(OS).infer_points(TS);
                                PrePrX_eq1=Boat(OS).PrX(TS);
                            end
                            pos_current(1)=round((Boat(TS).pos(1)+MapSize(1)*1852)/Res)+1;
                            pos_current(2)=round((Boat(TS).pos(2)+MapSize(2)*1852)/Res)+1;
                            row_index=ismember(PrRR_points,pos_current,'rows');
                            row_current=find(row_index==1);
                            if isempty(row_current)
                                disp('错误！当前点不在上一步预测点中');
                                break
                            end
                            % 得到公式（2）右边第一个乘数Pr(Xi=xi)，是一行
                            PrXTheta=PrePrX_eq1(row_current,:);  % 当前点在上一个时刻的所有theta的PrX值
                            PrTheta = PrTheta0.*PrXTheta;
                            PrTheta = PrTheta/sum(PrTheta(:));  % 公式（2）更新PrTheta
                            % 更新当前时刻的ThetaList0，并存到Boat(OS).Theta_his中
                            ThetaList0(:,6)=PrTheta';
                            Boat(OS).Theta_his=[Boat(OS).Theta_his;ThetaList0];
                            disp('  完成Equ2的计算，更新当前的Theta预测分布');
                            DisPrTheta=[Theta,PrTheta']
                            %% 步骤3.根据意图概率生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
                            % 步骤3.1. 依照更新后的PrTheta，利用MC方法生成theta点的阵列
                            N=100;
                            En=0.005;     %控制随机偏离，一次线性，熵
                            He=0.1;       %控制随机偏离，二次线性，超熵
                            Theta_MC=zeros(N,2);
                            for i_MC=1:N    %对生成的每一个点
                                % PP=[0.1 0.2 0.7];
                                n_Theta=length(PrTheta);
                                MCsub=zeros(1,n_Theta);
                                MCsub(1)=PrTheta(1);
                                for i=2:1:n_Theta
                                    MCsub(i)=MCsub(i-1)+PrTheta(i);
                                end
                                Select=find(MCsub>=rand);
                                row_select=Select(1);
                                %得到依MC方法和依正态分布生成的Theta_MC阵列，此时Theta_MC坐标仍然为实际坐标
                                Ennx=randn(1)*He+En;
                                Theta_MC(i_MC,1)=randn(1)*Ennx+Theta(row_select,1);
                                Enny=randn(1)*He+En;
                                Theta_MC(i_MC,2)=randn(1)*Enny+Theta(row_select,2);
                            end
                            disp('  完成步骤3.根据意图概率生成每一个路径点的点数');
                            %% 步骤4.每一个点云的位置作为终点，终点坐标输入FM，生成n条路径
                            MC_end(:,1)=round((Theta_MC(:,2)+MapSize(2)*1852)/Res)+1;
                            MC_end(:,2)=round((Theta_MC(:,1)+MapSize(1)*1852)/Res)+1;
                            [MCMap, MC_paths] = FMM(FM_map,start_point',MC_end');
                            % 步骤4.2. 每条路径归到栅格点上，绘制马赛克图
                            count_map=zeros(m,n);
                            for n_count=1:1:size(MC_end,1)    %针对MC之后的每一个仿真路径
                                count_map0=zeros(m,n);
                                L0=MC_paths{n_count};
                                L0=rot90(L0',2);
                                x0=L0(1:50,1); %规划的路径平滑处理
                                y0=L0(1:50,2);
                                %平滑两次
                                y_new=smooth(y0);
                                x_new=smooth(x0);
                                x_new1=smooth(x_new);
                                y_new1=smooth(y_new);
                                
                                L0(1:50,1)=x_new1;
                                L0(1:50,2)=y_new1; %得出最终的平滑后的L0
                                % 计算每一个L0的风险积分InL0
                                % 1)L0坐标点向下取到所在的栅格坐标，删去重复的，防止一个格子算两遍
                                L0_point0=floor(L0);    %L0坐标点向下取到所在的栅格坐标
                                L0_point=unique(L0_point0,'rows','stable');  %删去重复行，保留原顺序
                                row_count=L0_point(:,1);
                                col_count=L0_point(:,2);
                                count_map0(sub2ind(size(count_map0),row_count,col_count))=1;
                                % 最终得到的count_map就是MC之后每一个格子里的值，都是整数
                                count_map=count_map+count_map0;
                            end
                            % 步骤4.3.把count_map再回归到OS的船头和船尾，判断CAL
                            % 即count_map加上OS的航行遮罩
                            % 位置换成OS的
                            Boat_x=Boat(OS).pos(1,1);
                            Boat_y=Boat(OS).pos(1,2);
                            Boat_theta=-Boat(OS).COG_rad; %此处为弧度制
                            CAL_last=Boat(OS).CAL(TS);
                            alpha=45;
                            [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
                            Boat(OS).CAL(TS)=CAL_current;
                            disp('  完成步骤4.生成n条MC路径');
                        end
                    end
                    Boat(OS).infer_label(TS)=Boat(OS).infer_label(TS)+1;
                end
                Boat0=Boat;
                Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t);
                clear Boat0
            end
        end
    end
    
    if t==1000
        Boat1000=Boat;
        t1000=t_count12;
        save('data0306-0100','Boat1000','t1000');
        disp('1000s数据已保存');
        clear Boat1000
    elseif t==1500
        Boat1500=Boat;
        t1500=t_count12;
        save('data0306-0100','Boat1500','t1500','-append');
        disp('1500s数据已保存');
        clear Boat1500
    elseif t==2500
        Boat2500=Boat;
        t2500=t_count12;
        save('data0306-0100','Boat2500','t2500','-append');
        disp('2500s数据已保存');
        clear Boat2500
    elseif t==3500
        Boat3500=Boat;
        t3500=t_count12;
        save('data0306-0100','Boat3500','t3500','-append');
        disp('2000s数据已保存');
        clear Boat3500
    end
    t_count12=toc;    %时间计数
    disp([num2str(t),'时刻的所有船舶的运行时间: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['本次运行总时间: ',num2str(t3)]);

Boat_end=Boat;
t_end=t_count12;
save('data0306-0100','Boat_end','t_end','-append');
disp('最终数据已保存');
clear Boat_end
