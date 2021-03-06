%% 第一阶段的最终程序V3.5，没有成功，不知道错哪里了
% 核心思想1：路径点一旦确定，就不动了，就是多船避碰，不要考虑有没有碰撞风险的事
% 核心思想2：每一艘船有8个路径点，对应8个场景，就是8个门??再加上不采取措施的情况即直接去终点，总共9个门，并且应该有一些门距离很近导致不好判断
% 第1步--不推测的情况下，实现思想1
%              尝试与风险有关的路径点测试
%              注意：CAL的场景定义以图4.1-1为标准，本船眼中，场景000的意思是，所有船都是对本船让路，本船从对方船头过，CAL场是大扇形
% 第2步--在判断当前无风险时，不再重新决策，沿用之前的路径
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
% close all
datatime=datestr(now,'mmdd-HHMM');
datatitle=strcat('data',datatime);
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
% 不遵守COLREGs的CAL
CAL1=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];
% 根据初始场景设置的本船在两种状态下需要优先避让的船，写函数识别太麻烦，先人工标注出来了
% AvoS(1,1)表示1号船在场景1下优先避让的船，AvoS(1,2)表示1号船在场景2下优先避让的船
% 写函数的思路，场景1即找到本船逆时针顺序的第一艘目标船，场景2即找到本船顺时针顺序的第一艘目标船
AvoS=[3 4
    4 3
    2 1
    1 2];

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
    Boat(i).DecHis = [];
    Boat(i).HisPos=[];
    %上一次决策的运算结果
    Boat(i).path = [];
    Boat(i).infercount=0;
end

%其他决策参数设置
for i=1:1:Boat_Num
    %把目标点位置归一到栅格位置上，但是依然有正负
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)的初始目标位置，单位为米
    Boat(i).goal_real=goal0;
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    Boat(i).waypoint=[];
    Boat(i).centre=[];
    Boat(i).WayPoint_temp1=[];
    Boat(i).WP1=[];
    Boat(i).WP2=[];
    Boat(i).End=[];
    Boat(i).drawmap=[];
    Boat(i).FM_lable=0; %初始时刻FM_lable为0，直到第一个时刻计算FM
    % 每艘船的decision_label用于判断当前是否决策，如果没有异常，则按照上次决策内容继续航行。
    % 具体说，就是每次决策，先用贝叶斯判断，判断没问题，则CAL不变
    % 然后在决策时如果没有风险，且CAL不变，那么decision_label=0，即不决策
    % 在下一次更新状态是，还是按照上一次的决策来。
    Boat(i).decision_label=0;
    Boat(i).decision_count=0;   %决策计数
    Boat(i).Dechis.data=[];
    Boat(i).Dechis.startpos=[];
    Boat(i).Dechis.endpos=[];
    Boat(i).Dechis.map=[];
    Boat(i).Dechis.Scenariomap=[];
    Boat(i).Dechis.path=[];
    Boat(i).reachWP=0;
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
    % 场景的理解与判断，在当前阶段，每艘船只有正常，不符合避碰规则，失控三个状态：
    % 1.正常：服从COLREGs，此时按照Risk_value设置半径
    CAL_ts=CAL0(OS,:);
    WayPoint_temp1=[];
    WayPoint_label=[];
    k=1;
    for ts_i=1:1:Boat_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
        if ts_i~=OS
            %此处以图4.1-1为标准，本船眼中，场景000的意思是，所有船都是对本船让路，本船从对方船头过，CAL场是大扇形
            if CAL_ts(ts_i)==0
                WayPoint_temp1(k,:) = Boat(OS).WP1(k,:);  %船头路径点
            elseif CAL_ts(ts_i)==1
                WayPoint_temp1(k,:) = Boat(OS).WP2(k,:);  %船尾路径点
            end
            if ts_i==AvoS(OS,1)
                WayPoint_label(k)=1;
            else
                WayPoint_label(k)=0;
            end
            k=k+1;
        end
    end
    x1=WayPoint_temp1(1,1);    y1=WayPoint_temp1(1,2);
    x2=WayPoint_temp1(2,1);    y2=WayPoint_temp1(2,2);
    x3=WayPoint_temp1(3,1);    y3=WayPoint_temp1(3,2);
    %如果三点共线,就把第一个点稍微往(0,0)挪一下
    if abs((y3-y1)*(x2-x1)-(y2-y1)*(x3-x1))<=0.000001
        WayPoint_temp1(1,1)=WayPoint_temp1(1,1)+20;
        dis([num2str(OS),'船状态1的三个路径点共线，修正点']);
    end
    [WayPoint_OS,centre] =ScenarioWaypoint( WayPoint_temp1,WayPoint_label);
    Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS];
    Boat(OS).centre=[Boat(OS).centre;centre];
    Boat(OS).WayPoint_temp1=[Boat(OS).WayPoint_temp1;WayPoint_temp1];
    % 2.反常：不服从COLREGs，此时按照Risk_value2设置半径
    CAL_ts=CAL1(OS,:);
    WayPoint_temp1=[];
    WayPoint_label=[];
    k=1;
    for ts_i=1:1:Boat_Num               %按照场景的2进制编码提取出每一艘船在当前场景下的路径点
        if ts_i~=OS
            %此处以图4.1-1为标准，本船眼中，场景000的意思是，所有船都是对本船让路，本船从对方船头过，CAL场是大扇形
            if CAL_ts(ts_i)==0
                WayPoint_temp1(k,:) = Boat(OS).WP1(k,:);  %船头路径点
            elseif CAL_ts(ts_i)==1
                WayPoint_temp1(k,:) = Boat(OS).WP2(k,:);  %船尾路径点
            end
            if ts_i==AvoS(OS,2)
                WayPoint_label(k)=1;
            else
                WayPoint_label(k)=0;
            end
            k=k+1;
        end
    end
    x1=WayPoint_temp1(1,1);    y1=WayPoint_temp1(1,2);
    x2=WayPoint_temp1(2,1);    y2=WayPoint_temp1(2,2);
    x3=WayPoint_temp1(3,1);    y3=WayPoint_temp1(3,2);
    %如果三点共线,就把第一个点稍微往(0,0)挪一下
    if abs((y3-y1)*(x2-x1)-(y2-y1)*(x3-x1))<=0.000001
        WayPoint_temp1(1,1)=WayPoint_temp1(1,1)+20;
        dis([num2str(OS),'船状态2的三个路径点共线，修正点']);
    end
    [WayPoint_OS,centre]=ScenarioWaypoint(WayPoint_temp1,WayPoint_label);
    Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS];
    Boat(OS).centre=[Boat(OS).centre;centre];
    Boat(OS).WayPoint_temp1=[Boat(OS).WayPoint_temp1;WayPoint_temp1];
    % 3.失控：直接向着目标点
    Boat(OS).waypoint=[Boat(OS).waypoint;Boat(OS).goal_real];
    %完成对当前OS所有3个路径点的收集
end
%识别初始场景，即所有船都符合避碰规则时
for  OS=1:1:Boat_Num
    Boat(OS).currentWP=Boat(OS).waypoint(1,:);
end
t_wp=toc;
risk_factor_count=0;
%% 正式决策开始
for t=1:1:4000   %tMax*2
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
        %判断当前i时刻是在OS船的决策周期中,compliance==1即本船正常,且未到达目标点
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(OS).reach==1
            disp([num2str(t),'时刻',num2str(OS),'船开始决策']);
            %% 目标船舶的路径点贝叶斯预测，确定真实的CAL
            if shipLabel(OS,2)==0    % inferLabbel:是否推测:0.不推测,1.推测;
                % 不推测，即不改变CAL，即按照最初的CAL，每艘船都知道彼此的CAL
                Boat(OS).CAL_infer=CAL0(OS,:); % Boat(OS).CAL_infer是一个向量，表示OS对所有TS的CAL
            elseif  shipLabel(OS,2)==1
                Boat(OS).infercount=Boat(OS).infercount+1;
                disp([' ',num2str(OS),'船第',num2str(Boat(OS).infercount),'次推测']);
                for  TS=1:1:Boat_Num
                    
                    if TS~=OS
                        disp(['  进入',num2str(TS),'船视角']);
                        TSinferlabel=[t,Boat(OS).InferData(TS).infer_label(end,2)+1];
                        Boat(OS).InferData(TS).infer_label=[Boat(OS).InferData(TS).infer_label;TSinferlabel];
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
                        % 推测方法：贝叶斯推测（测试通过后变成函数）
                        % 具体步骤：
                        %        步骤1.公式(1)更新当前位置分布
                        %        步骤2.公式(2)更新当前的意图分布
                        %        步骤3.根据意图概率生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
                        %        步骤4.每一个点云的位置作为终点，终点坐标输入FM，生成n条路径
                        %        步骤5.每一个路径回归到点，每一个路径点计数
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %% 步骤0.推测前的准备工作
                        % 包括：
                        % 步骤0.1. 找到TS视角下所有可能的路径点
                        % 步骤0.2. 绘制当前TS眼中的风险场、规则场和引导场
                        ScenarioMap=zeros(m,n);
                        RRT_map=zeros(m,n);
                        % 步骤0.1. 找到TS视角下所有可能的路径点
                        % 设定：一艘船有可能有风险不规避，即为失控船，但是不可能没有风险还无谓的规避。
                        %            一旦目标船没有风险船了，目标就只有自身的终点
                        ThetaState=[];
                        if  Boat(OS).InferData(TS).infer_label(end,2)==1
                            % TS之前没有推测过，这是第一次，需要初始化PrTheta
                            %注意0.1.中 第一行是TS的目标点，如果TS不决策，也就是直接向着这一点去
                            % 与其他非符合避碰规则的场景共同为0.5
                            ThetaState=[t,OS,TS,Boat(TS).goal,0.5/8,Boat(OS).InferData(TS).infer_label(end,2)];
                            %第一次推测，因此使用预设的PrTheta
                            for infer_point=1:8
                                WP=Boat(TS).waypoint(infer_point,:);
                                if  isequl(Boat(TS).currentWP,WP)
                                    Pr=0.5;      %符合避碰规则的场景预设为0.5
                                else
                                    Pr=0.5/8;
                                end
                                ThetaState  =[ThetaState;
                                    t,OS,TS,WP,Pr,Boat(OS).InferData(TS).infer_label(end,2)];
                            end
                        else
                            % 已经有了预测的记录后，采用上一个时刻的PrTheta，只推测和预测
                            % 从历史记录中提取出新的ThetaList0
                            for k_inf=1:1:size(Boat(OS).Theta_his,1)
                                %遍历Boat(OS).Theta_his找到上一次推测TS的数据
                                if Boat(OS).Theta_his(k_inf,3)==TS && ...
                                        Boat(OS).Theta_his(k_inf,7)==Boat(OS).InferData(TS).infer_label(end-1,2)
                                    % 首先取出TS自身的目标点的推测值
                                    ThetaState  =[ThetaState;
                                        Boat(OS).Theta_his(k_inf,1:6),Boat(OS).InferData(TS).infer_label(end,2)];
                                end
                            end
                        end
                        % 步骤0.2. 绘制当前TS眼中的风险场、规则场和引导场
                        % 计算避碰规则下的风险场，规则场RuleField
                        Boat_theta = -Boat(ts_infer).COG_rad; %此处为弧度制
                        Boat_Speed =  Boat(ts_infer).SOG;
                        Shiplength =  ShipSize(ts_infer,1);
                        % 风险场
                        SCR_temp = ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                        cro_angle= abs(Boat(TS).COG_deg-Boat(ts_infer).COG_deg);
                        RRT_map=RRT_map+SCR_temp;
                        Rule_eta=2;
                        Rule_alfa=0.1;
                        % 规则场
                        CAL_Field0=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,0);
                        CAL_Field1=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,1);
                        % 规则场是两种状态按照概率的叠加，如果状态为0.5/0.5，则生成的路径不具有参考意义
                        CAL_Field=Pr1*CAL_Field0+Pr2*CAL_Field1;
                        % 总的环境场
                        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
                        
                        RiskMap=1./(ScenarioMap+1);
                        %完成对TS所有可能theta的收集（步骤0.1.1完成）
                        Theta=ThetaState(:,4:5); %按顺序提取当前所有的theta点
                        %Theta坐标离散化
                        Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res)+1;
                        Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res)+1;
                        disp(['    完成对',num2str(TS),'船所有',num2str(size(Theta,1)),'个可能theta的收集']);
                        
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
                        %                             FM_map=RiskMap;
                        % TS的FMM初始位置
                        start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
                        start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
                        %如果start_point在FM_map的值<0.001，则FM的起始点陷入0点，无法计算路径，要加0.001
                        if FM_map(start_point(1,1),start_point(1,2))<0.001
                            FM_map=FM_map+0.01;
                        end
                        % FM_map是FM算法的输入矩阵，最大值为1，因此大于1时需要强制置为1
                        FM_map(FM_map>1)=1;
                        [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
                        % 注意：得出的InferMap和L0_paths的都是先验的
                        % 1.InferMap是FMM方法得出的以当前TS位置为起点的上一个时刻的CAL概率为先验概率的地图
                        % 2.L0_paths是当前的所有theta得出的初步的路径
                        
                        %% 步骤1.公式(1)计算上一个时刻的位置分布，给Eq2的计算使用
                        % 在步骤0中找到了当前时刻需要的Theta，在本步找到所有的可达点RR_points;
                        % 由于在上一时刻，不知道下次什么时候推测，因此没法一次找到全部的可达点。
                        % 在这个版本中改成，每次计算上一步推测之后到当前的所有的可达点。这个可达点的集合可能多也可能少
                        disp('    开始Equ1的计算');
                        t_eq11=toc;
                        % 此时的输入为TS上一步的状态,先找到上一步推测的时刻，再从HisPos中检索出当时的位置
                        if   Boat(OS).InferData(TS).infer_label(end-1,1)==0
                            % 为0指初始设置，之前没有推测过，则就近提取上一步的HisPos
                            t_last=t-1;
                        else
                            % 不为0则之前推测过，则提取上一次推测时的HisPos
                            t_last=Boat(OS).InferData(TS).infer_label(end-1,1);
                        end
                        Boat_x0    = Boat(TS).HisPos(t_last,1);
                        Boat_y0    = Boat(TS).HisPos(t_last,2);
                        Boat_theta=-Boat(TS).HisCOG(t_last,1);
                        delta_t=t-t_last;
                        speed=Boat(TS).speed;
                        [PrRR_points,PrePrX_eq1] = BayesEqu1(Boat_x0,Boat_y0,Boat_theta,speed,delta_t,L0_paths,Theta,Theta_end,FM_map,MapSize,Res);
                        % Boat(OS).inferdata中存储的都是最近一步的数据，给下一次用
                        Boat(OS).InferData(TS).infer_points=PrRR_points;
                        Boat(OS).InferData(TS).PrX=PrePrX_eq1;
                        t_eq12=toc;
                        disp(['    完成Equ1的计算，更新当前的位置分布，用时',num2str(t_eq12-t_eq11)]);
                        %% 步骤2.公式(2)更新当前的意图分布
                        pos_current(1)= round((Boat(TS).pos(1)+MapSize(1)*1852)/Res)+1;
                        pos_current(2)= round((Boat(TS).pos(2)+MapSize(2)*1852)/Res)+1;
                        row_index=ismember(PrRR_points,pos_current,'rows');
                        row_current=find(row_index==1);
                        if isempty(row_current)
                            disp('错误！当前点不在上一步预测点中，程序暂停');
                        end
                        % 得到公式（2）右边第一个乘数Pr(Xi=xi)，是一行
                        PrXTheta=PrePrX_eq1(row_current,:);  % 当前点在上一个时刻的所有theta的PrX值
                        % 得到公式（2）右边第二个乘数Pr(theta0)，即上一步的公式（2）的值
                        PrTheta0=ThetaState(:,6)/sum(ThetaState(:,6)); %PrTheta归一化，防止出现越来越小的情况
                        PrTheta0=PrTheta0';        %第一次预测，第一行是初始的针对每一个theta的先验概率
                        PrTheta = PrTheta0.*PrXTheta;
                        % 公式（2）更新PrTheta
                        PrTheta = PrTheta/sum(PrTheta(:));
                        
                        % 更新当前时刻的ThetaList0，并存到Boat(OS).Theta_his中
                        ThetaState(:,6)=PrTheta';
                        Boat(OS).Theta_his=[Boat(OS).Theta_his;ThetaState];
                        disp('    完成Equ2的计算，更新当前的Theta预测分布');
                        %% 步骤3.根据意图概率生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
                        % 步骤3.1. 依照更新后的PrTheta，利用MC方法生成theta点的点云
                        t_eq31=toc;
                        PointCloud_N=200;
                        Theta_MC=zeros(PointCloud_N,2);
                        for i_MC=1:PointCloud_N    %对生成的每一个点
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
                            Theta_MC(i_MC,1)=Theta(row_select,1);
                            Theta_MC(i_MC,2)=Theta(row_select,2);
                        end
                        t_eq32=toc;
                        disp(['    完成Equ3的计算，根据意图概率生成对应每一个Theta的目标点云，用时',num2str(t_eq32-t_eq31)]);
                        %% 步骤4.每一个点云的位置作为终点，计算PRM
                        t_eq41=toc;
                        % 步骤4.0.绘制当前的地图，船舶周围为障碍物，其他都是可走的区域
                        count_map=zeros(m,n);
                        RRTstart(1)=round(abs(( Boat(TS).pos(1)+MapSize(1)*1852)/Res))+1;
                        RRTstart(2)=round(abs((-Boat(TS).pos(2)+MapSize(2)*1852)/Res))+1;
                        RRTbraeklable=0;
                        for n_count=1:1:size(Theta_MC,1)    %针对MC之后的每一个仿真路径
                            % 步骤4.1.对每一个Theta_MC，当前位置为起点，Theta_MC为终点，计算PRM
                            RRTend(1)=round(abs(( Theta_MC(n_count,1)+MapSize(1)*1852)/Res))+1;
                            RRTend(2)=round(abs((-Theta_MC(n_count,2)+MapSize(2)*1852)/Res))+1;
                            PrFocus=0.2+0.8*rand(1);
                            [L0,braeklable] = RRTPlanner(RRT_map,RRTstart,RRTend,speed,PrFocus) ;
                            RRTbraeklable=RRTbraeklable+braeklable;
                            count_map0=zeros(m,n);
                            % 计算每一个L0的风险积分InL0
                            % 1)L0坐标点向下取到所在的栅格坐标，删去重复的，防止一个格子算两遍
                            L0_point0=floor(L0);    %L0坐标点向下取到所在的栅格坐标
                            L0_point=unique(L0_point0,'rows','stable');  %删去重复行，保留原顺序
                            row_count=L0_point(:,1);
                            col_count=L0_point(:,2);
                            % 步骤4.2. 每条路径归到栅格点上，绘制马赛克图
                            count_map0(sub2ind(size(count_map0),row_count,col_count))=1;
                            % 最终得到的count_map就是MC之后每一个格子里的值，都是整数
                            count_map=count_map+count_map0;
                        end
                        Boat(OS).InferData(TS).infermap(t).map=count_map;
                        % 步骤4.3.把count_map再回归到OS的船头和船尾，判断CAL
                        % 即count_map加上OS的航行遮罩
                        % 位置换成OS的
                        Boat_x    = Boat(OS).pos(1,1);
                        Boat_y    = Boat(OS).pos(1,2);
                        Boat_theta=-Boat(OS).COG_rad; %此处为弧度制
                        CAL_last  = Boat(OS).CAL_infer(TS);  %此处CAL是一个值，当前推测的OS对TS的CAL
                        alpha=45;
                        [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
                        Boat(OS).CAL_infer(TS)=CAL_current;  %此处CAL是一个值，当前推测的OS对TS的CAL
                        t_eq42=toc;
                        disp(['    完成Equ4的计算，生成',num2str(PointCloud_N),'条MC路径，用时',num2str(t_eq42-t_eq41)]);
                        disp(['    完成对',num2str(TS),'船的推测，对TS新的的CAL为',num2str(CAL_current),'（CAL0=）',num2str(CAL0(OS,TS))]);
                    end
                end
                figure
                infer_map=zeros(m,n);
                for ts_map=1:1:Boat_Num
                    if ts_map~=OS &&  ismember(t,Boat(OS).InferData(ts_map).infer_label)
                        infer_map=infer_map+Boat(OS).InferData(ts_map).infermap(t).map;
                    end
                end
                % 显示马赛克图,检验当前的count_map
                ss=pcolor(Y,X,infer_map);  %注意这里Y，X是相反的
                set(ss, 'LineStyle','none');
                colorpan=ColorPanSet(0);
                colormap(colorpan);%定义色盘
                hold on
                for plotship=1:1:4
                    %WTF:画出船舶的初始位置
                    ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
                    %WTF:画出船舶的结束位置
                    ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
                    %WTF:画出过往的航迹图
                    plot(Boat(plotship).HisPos(:,1),Boat(plotship).HisPos(:,2),'k.-');
                end
                hold on
                % 显示Theta位置
                plot(Theta(:,1),Theta(:,2),'r*')
                axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
                set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
                set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
                set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
                set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
                grid on;
                xlabel('\it n miles', 'Fontname', 'Times New Roman');
                ylabel('\it n miles', 'Fontname', 'Times New Roman');
                box on;
            end
            
            %% 进入本船决策，采用生成了目标路径点的FM算法
            Boat(OS).CAL=Boat(OS).CAL_infer;
            %                 Boat0=Boat;
            % 更新了的只是OS的内容
            %                 Boat = MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t);
            % function Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t)
            % 决策函数，包括当前的FM输入地图绘制（风险地图，规则地图，引导图）
            % 此时Boat只更新了Boat(OS)的内容，决策也只是OS针对所有TS的
            % Boat=Boat0;
            %% 建立本船的航行场
            % 绘制当前本船眼中目标船的SCR
            t_count31=toc;    %时间计数
            ScenarioMap=zeros(m,n);
            RiskLabel=[];
            Risk_temp=[];
            k=1;
            k_ship=1;
            if   Boat(OS).decision_count==0     %如果本船没有决策过，第一个决策周期还是要决策的
                Boat(OS).decision_label=1;
            else
                Boat(OS).decision_label=0;
            end
            for TS=1:1:Boat_Num
                if TS~=OS
                    v_os = Boat(OS).speed(end,:);
                    course_os = Boat(OS).COG_deg(end,:);
                    pos_os = Boat(OS).pos;
                    v_ts = Boat(TS).speed(end,:);
                    course_ts = Boat(TS).COG_deg(end,:);
                    pos_ts = Boat(TS).pos;
                    dis_now=norm(pos_ts-pos_os);
                    Dis0=2*1852;% 只有两船的距离小于2海里时，才开始绘制地图
                    % 这样，即使目标船已经和本船背离，在FM路径规划的时候也不会把目标船考虑进去
                    if  dis_now<Dis0
                        disp(['  ',num2str(OS),'号船与',num2str(TS),'号船距离为',num2str(dis_now),'需要计算目标船势场']);
                        Boat(OS).decision_label=Boat(OS).decision_label+1;
                        Boat(OS).drawmap=[Boat(OS).drawmap;t  TS];
                        % 为了修正路径点在FM地图中的畸变，需要把路径点附近的船舶风险场变小
                        DisWP_temp=pos_os-Boat(OS).currentWP;
                        DisWP=norm(DisWP_temp);
                        risk_factor=1;
                        if  DisWP>2*Res   %未达到路径点，目标点为终点
                            DisWP_temp_ts=pos_ts-Boat(OS).currentWP;
                            DisWP_ts=norm(DisWP_temp_ts);
                            if  DisWP_ts<=2*1852   %TS与WP的距离小于1nm时，就在TS风险范围中了，需要减小TS的影响
                                risk_factor=0.001;
                                disp(['    目标船',num2str(TS),'距离目标路径点',num2str(DisWP_ts),'修正风险场']);
                                risk_factor_count=risk_factor_count+1;
                            end
                        end
                        Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(TS,1);
                        SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                        
                        %计算避碰规则下的风险场，规则场RuleField
                        cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
                        % 这个CAL是OS对TS的CAL，为0或1
                        CAL=Boat(OS).CAL(TS);
                        Rule_eta=2;
                        Rule_alfa=0.1;
                        CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
                        ScenarioMap=ScenarioMap+(SCR_temp+CAL_Field)*risk_factor;
                    end
                end
            end
            RiskMap=1./(ScenarioMap+1);
            
            %% 目标点确定
            if Boat(OS).reachWP==0   % reachWP==0说明还没到路径点，此时目标点为路径点
                %    判断方法：如果已经到达场景路径点，则改为最终的目标点
                DisWP_temp=Boat(OS).pos-Boat(OS).currentWP;
                DisWP=norm(DisWP_temp);
                if  DisWP<4*Res
                    end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
                    end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
                    endpos_real=Boat(OS).goal;
                    disp('    已到达路径点，目标点为终点');
                    Boat(OS).reachWP=1;%到达路径点后，reachWP=1
                    Boat(OS).decision_label=Boat(OS).decision_label+1; % 在更新目标点时，decision_label变化一次
                else
                    end_point(1,2) =round((Boat(OS).currentWP(1,1)+MapSize(1)*1852)/Res)+1;
                    end_point(1,1) =round((Boat(OS).currentWP(1,2)+MapSize(2)*1852)/Res)+1;
                    endpos_real=Boat(OS).currentWP;
                    disp('    避碰中，目标点为场景路径点');
                end
            else        %到达路径点后，目标点改为终点，由于reachWP只改了一次，此后不会再变
                end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
                end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
                endpos_real=Boat(OS).goal;
                disp('    已经过路径点，目标点为终点');
            end
            Boat(OS).End=[Boat(OS).End;t,endpos_real,end_point];
            if Boat(OS).decision_label~=0 % Boat(OS).decision_label不为0时才决策，否则不决策
                %% 绘制当前本船的航行遮罩
                Boat_x=Boat(OS).pos(1,1);
                Boat_y=Boat(OS).pos(1,2);
                Boat_theta=-Boat(OS).COG_rad(end,:); %此处为弧度制
                % Shiplength = ShipSize(OS,1);
                alpha=30;    %30度在2*18.52的分辨率上太小了，在最后的AG_map上开口处会有一个诡异的尖刺，是由于开口附近的两个方格连在一起了
                R=500;
                AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                [AG_row,AG_col]=find(AFMfield~=0);
                AG_points=[AG_row,AG_col];
                AG_map0=ones(size(AFMfield));
                [AG_map, ~] = FMM(AG_map0, AG_points');
                FM_map=min(AG_map,RiskMap);
                start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
                start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
                %如果start_point在FM_map的值<0.001，则FM的起始点陷入0点，无法计算路径，要加0.001
                if FM_map(start_point(1,1),start_point(1,2))<0.001
                    FM_map=FM_map+0.01;
                end
                % FM_map是FM算法的输入矩阵，最大值为1，因此大于1时需要强制置为1
                FM_map(FM_map>1)=1;
                t_count32=toc;
                disp(['  ',num2str(OS),'号船计算航行场用时: ',num2str(t_count32-t_count31)]);
                %% FM算法主程序
                
                %2. 路径规划
                %当起始点或终点到边界处时，认为在内部，没有出现过这种情况，但是懒得删了
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
                
                [~, paths] = FMM(FM_map,start_point',end_point');
                t_count22=toc;
                disp(['      ',num2str(OS),'号船路径规划用时: ',num2str(t_count22-t_count21)]);
                Boat(OS).FM_lable=Boat(OS).FM_lable+1;
                %3. 数据处理
                FMpath = paths{:};
                path0=rot90(FMpath',2);
                Boat(OS).AFMpath=path0;
                posData_temp = zeros(size(path0));
                posData_temp(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                posData_temp(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                
                if Boat(OS).pos(1)~=posData_temp(1,1) && Boat(OS).pos(2)~=posData_temp(1,2)
                    % 由于栅格的存在，一般规划出的path的起点并不是决策的起点的实际位置
                    % 则需要把路径平滑拟合到规划的path上
                    %                     posData=[Boat(OS).pos;posData];
                    startlines=30;
                    for  i_path=1:1:startlines
                        x_p0=posData_temp(startlines+1,1);
                        y_p0=posData_temp(startlines+1,2);
                        x_p1=posData_temp(1,1);
                        y_p1=posData_temp(1,2);
                        x_p2=posData_temp(i_path,1) ;
                        y_p2=posData_temp(i_path,2);
                        
                        x_p10=Boat(OS).pos(1);
                        y_p10=Boat(OS).pos(2);
                        if  abs(x_p1-x_p0)>0.1
                            lambda= (x_p2-x_p0)/(x_p1-x_p0);
                        else
                            lambda= (y_p2-y_p0)/(y_p1-y_p0);
                        end
                        x_p20=lambda*(x_p10-x_p0)+x_p0;
                        y_p20=lambda*(y_p10-y_p0)+y_p0;
                        
                        posData(i_path,1)=x_p20;
                        posData(i_path,2)=y_p20;
                    end
                    posData=[posData;posData_temp(startlines+1:end,:)];
                end
                
                Boat(OS).path=posData;     %如果decision_label==0，则 Boat(OS).path保持原状
                Boat(OS).Current_row=1;   %每次重新决策，当前行数置1
                Boat(OS).decision_count=Boat(OS).decision_count+1;
                iDec=Boat(OS).decision_count;
                Boat(OS).DecHis = [Boat(OS).DecHis;t,iDec,Boat_x,Boat_y,start_point,endpos_real,end_point];
%                 Boat(OS).Dechis(iDec).data=[t,iDec,start_point,end_point];
%                 Boat(OS).Dechis(iDec).startpos=[Boat_x,Boat_y];   %采用真实位置，不用栅格化数据反算的，以减小误差
%                 Boat(OS).Dechis(iDec).endpos=endpos_real;
%                 Boat(OS).Dechis(iDec).map=FM_map;
%                 Boat(OS).Dechis(iDec).Scenariomap=ScenarioMap;
%                 Boat(OS).Dechis(iDec).path=posData;
            end
        end
        
    end
    t_count12=toc;    %时间计数
    disp([num2str(t),'时刻的所有船舶的运行时间: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['本次运行总时间: ',num2str(t3)]);

t_end=t_count12;
save(datatitle,'Boat');
disp('最终数据已保存');

