%% 贝叶斯绘图程序
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
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

tic;%tic1

%% 贝叶斯推测总体思路
% 推测的步骤
% 步骤1.公式(1)更新当前位置分布
% 步骤2.公式(2)更新当前的意图分布
% 步骤3.根据意图概率生成每一个路径点的点数，根据点数生成围绕每一个路径点的点云
% 步骤4.每一个点云的位置作为终点，终点坐标输入FM，生成n条路径
% 步骤5.每一个路径回归到点，每一个路径点计数

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

%% 数据加载与处理
%地图设置
% MapSize_temp=max(max(abs(Start_pos)))/1852;
load data0814-2107.mat

MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

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

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40    ];
t0=100;
OS=1;
TS=2;
Boat_Num=4;

for pic=1:3
    tfor0=toc;
    t=pic*t0+t0;
    disp(['第一个时刻',num2str(t),'计算']);
    %% 步骤0，准备工作，计算上一步的预测值
    ScenarioMap=zeros(m,n);
    RRT_map=zeros(m,n);
    v_os      = Boat(TS).speed;
    course_os = Boat(TS).HisCOG(t,2);
    pos_os    = Boat(TS).HisPos(t,:);
    % 步骤0.1.2. TS视角下所有可能的路径点
    % 成对出现，作为路径点的theta。
    % 设定：一艘船有可能有风险不规避，即为失控船，但是不可能没有风险还无谓的规避。
    %      一旦目标船没有风险船了，目标就只有自身的本船目标一点
    WP1 = Boat(TS).waypoint(1,:);  % 第1个点是遵守避碰规则的点
    WP2 = Boat(TS).waypoint(2,:);  % 第2个点是不遵守避碰规则的点
    WP3 = Boat(TS).waypoint(3,:);  % 第3个点是目标点
    if  size(Boat(OS).InferData(TS). infer_label,1)==1   %TS之前没有推测过，这是第一次
        % 步骤0.1. 找到所有theta
        % 当不是初始状态时，则直接提取沿用上个时刻的theta，只推测和预测，没有本步
        % 步骤0.1.1. 第一行是TS的目标点，如果TS不决策，也就是直接向着这一点去
        Boat(OS).InferData(TS). infer_label(1,1)=t;
        Boat(OS).InferData(TS). infer_label(1,2)=Boat(OS).InferData(TS). infer_label(1,2)+1;
        %第一次推测，因此使用预设的PrTheta
        Pr1=0.8;   %初始阶段，认为从WP1过，即遵守避碰规则的可能性大
        Pr2=0.1;
        Pr3=0.1;
        
        Boat(OS).InferData(TS).ThetaPr0=[Pr1,Pr2,Pr3];   %存放上一个时刻的推测Pr
        Boat(OS).InferData(TS).ThetaPr=[Pr1,Pr2,Pr3];   %存放最新的推测Pr
        Boat(OS).InferData(TS).Theta_his  =[ t,Boat(OS).InferData(TS). infer_label(1,2),Pr1,Pr2,Pr3]; %存放推测历史
    else    %已经有了预测的记录后，采用上一个时刻的PrTheta
        % 把Pr1,Pr2从上一个时刻继承到当前时刻
        infer_label_new=Boat(OS).InferData(TS). infer_label(end,2)+1;
        Boat(OS).InferData(TS). infer_label=[Boat(OS).InferData(TS). infer_label;t,infer_label_new];
        Boat(OS).InferData(TS).ThetaPr0=Boat(OS).InferData(TS).ThetaPr;   %存放上一个时刻的推测Pr
        
    end
    %完成对TS所有可能theta的收集（步骤0.1.1完成）
    disp('完成对TS所有可能theta的收集');
    Theta=[WP1;WP2;WP3]; %TS所有的theta点
    Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res);
    Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res);
    
    for ts_infer=1:1:Boat_Num
        Pr1= Boat(OS).InferData(TS).ThetaPr0(1);
        Pr2= Boat(OS).InferData(TS).ThetaPr0(2);
        v_ts      = Boat(ts_infer).speed;
        course_ts = Boat(ts_infer).HisCOG(t,2);
        pos_ts    = Boat(ts_infer).HisPos(t,:);
        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA_temp=CPA_temp(5);
        TCPA_temp=CPA_temp(6);
        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1即排除啊当前位置为CPA的情况
            CurrentRisk_TS=1; %有碰撞风险为1
        else
            CurrentRisk_TS=0;
        end
        if ts_infer~=TS && CurrentRisk_TS==1 %TS视角下，ts_infer(包括本船OS)和TS的确有碰撞风险
            % 步骤0.2. 绘制当前TS眼中的风险场、规则场和引导场
            % 计算避碰规则下的风险场，规则场RuleField
            Boat_theta = -Boat(ts_infer).HisCOG(t,1); %此处为弧度制
            Boat_Speed = Boat(ts_infer).SOG;
            % 风险场
            Shiplength = ShipSize(ts_infer,1);
            SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            cro_angle=abs(Boat(TS).HisCOG(t,2)-Boat(ts_infer).HisCOG(t,2));
            
            CAL=CAL0(TS,ts_infer);
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
        end
    end
    RiskMap=1./(ScenarioMap+1);
    
    % 当前TS的引导场
    Boat_x=Boat(TS).HisPos(t,1);
    Boat_y=Boat(TS).HisPos(t,2);
    Boat_theta=-Boat(TS).HisCOG(t,1); %此处为弧度制
    alpha=30;
    R=500;
    
    AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
    [AG_row,AG_col]=find(AFMfield~=0);
    AG_points=[AG_row,AG_col];
    
    AG_map0=ones(size(AFMfield));
    [AG_map, ~] = FMM(AG_map0, AG_points');
    FM_map=min(AG_map,RiskMap);
    % TS的FMM初始位置
    start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res);
    start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res);
    step_length=[Res,Res];
    while  FM_map(start_point(1),start_point(2))<0.001
        start_temp = ang_point(Boat_x,Boat_y,Boat(OS).COG_deg,step_length);
        start_point(1,2) = round((start_temp(1)+MapSize(1)*1852)/Res);
        start_point(1,1) = round((start_temp(2)+MapSize(2)*1852)/Res);
        step_length=step_length+step_length;
    end
    t_count01=toc;
    [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
    t_count02=toc;
    % 注意：得出的InferMap和L0_paths的都是先验的
    % 1.InferMap是FMM方法得出的以当前TS位置为起点的上一个时刻的CAL概率为先验概率的地图
    % 2.L0_paths是当前的所有theta得出的初步的路径
    disp(['   完成theta的初步路径L0_paths的收集，用时',num2str(t_count02-t_count01)]);
    
    %% 步骤1.公式(1)计算上一个时刻的位置分布，给Eq2的计算使用
    % 在步骤0中找到了当前时刻需要的Theta，在本步找到所有的可达点RR_points;
    % 由于在上一时刻，不知道下次什么时候推测，因此没法一次找到全部的可达点。
    % 在这个版本中改成，每次计算上一步推测之后到当前的所有的可达点。这个可达点的集合可能多也可能少
    disp('    开始Equ1的计算');
    t_eq11=toc;
    % 此时的输入为TS上一步的状态,先找到上一步推测的时刻，再从HisPos中检索出当时的位置
    if  size(Boat(OS).InferData(TS). infer_label,1)==1
        % 为1指初始设置，之前没有推测过，则就近提取上一步的HisPos
        t_last=t-t0;
    else
        % 不为0则之前推测过，则提取上一次推测时的HisPos
        t_last=Boat(OS).InferData(TS).infer_label(end-1,1);
    end
    Boat_x0    = Boat(TS).HisPos(t_last,1);
    Boat_y0    = Boat(TS).HisPos(t_last,2);
    Boat_theta=-Boat(TS).HisCOG(t_last,1);
    delta_t=t-t_last;
    speed=Boat(TS).speed;
    Boat_x=Boat_x0;
    Boat_y=Boat_y0;
    course=Boat_theta;
    
    % 贝叶斯推断的公式1，根据当前的位置和theta计算下一步的位置分布
    % 输入：Boat_x,Boat_y,Boat_theta,speed：都是TS船的状态
    %      Theta：第0步得出的theta点的坐标和概率
    %      MapSize,Res：地图信息
    % 输出：RR_points：下一个时刻所有的可达点，栅格形式，地图上每一点的坐标
    %      PrX_eq1：PrX的值，每一行代表一个可达点RR_points，每一列代表一个theta
    % 步骤1.   公式(1)更新当前位置分布
    %% 步骤1.1. 绘制L0，计算风险积分
    % 找到TS从起点到theta的路径L0
    Theta_L0=[];
    for k_theta=1:1:size(Theta,1)     %针对每一个theta
        %对L0数据处理,风险积分是在栅格上求的，所有的贝叶斯推断都是在栅格上求的
        L0 = L0_paths{k_theta};
        L0 = rot90(L0',2);
        L0_integral = RiskIntegral(L0,FM_map);
        % 得到每一个theta对应的当前的L0风险积分值
        Theta_L0=[Theta_L0,L0_integral];
    end
    t_res=Res/speed; %要保证每次至少能前进1格
    if delta_t>t_res
        %如果上次推测的时间到当前时刻，足以前进至少1格，则使用delta_t
        t_step=delta_t+3;  %再往下计算5s，至少算到下一个决策周期
    else
        %如果上次推测的时间到当前时刻太近，不足以前进1格，则使用t_res
        t_step=t_res+3;  %再往下计算5s，至少算到下一个决策周期
    end
    %% 步骤1.2. 找出t时刻所有的可达点Reachable points(Rb_points)
    % 用筛选AG_points的方法确定某一个时刻的可达点集合（r=V*(t-1),R=V*t）
    t_eq101=toc;
    % 第一个可达点是当前点，在两次推测过近的情况下，可能出现两次移动都在一个格子中的情况
    Rb_points(1,2)=ceil((Boat_x+MapSize(1)*1852)/Res);
    Rb_points(1,1)=ceil((Boat_y+MapSize(2)*1852)/Res);
    R_infer=speed*t_step;
    r_infer=0.8*R_infer;
    alpha=120;
    %RR_points和航行遮罩一样，得到的是地图上点的横纵坐标的n*2的矩阵
    RRpoint0=ReachableRange(Boat_x,Boat_y,course,alpha,R_infer,r_infer,MapSize,Res);
    %顺着往下接这个n*2的矩阵，因此不存在尺寸不统一的问题，最后得到l*2的矩阵，l为所有3步以内可达点的个数
    Rb_points=[Rb_points;RRpoint0];
    
    % 使用unique函数，删去相同的点（即相同的行）
    Rb_points=unique(Rb_points,'rows','stable');
    % unique(A,’stable’)去重复后不排序。默认的排序是unique(A,’sorted’)，’sorted’一般省略掉了。
    Rb_points=fliplr(Rb_points);
    Rb_points0=Rb_points;
    Rb_points=Rb_points0(2:end,:);
    while size(Rb_points,1)>70
        Rb_points=Rb_points(1:2:end,:);   %原来的可达点Rb_points太多了，1.3步根本算不下去，筛选到300个以内
    end
    t_eq102=toc;
    disp(['      1.2找到所有可达点，可达点原有',num2str(size(Rb_points0,1)),'个，现有',num2str(size(Rb_points,1)),'个，共用时',num2str(t_eq102-t_eq101)]);
    %% 步骤1.3. 绘制L1计算线积分
    % 对每一个RR_point找到对应的TS-Reachable-theta的路径L1
    t_eq131=toc;
    RP_all_L1=[];
    for k_rp=1:1:size(Rb_points,1)
        Reach_point_now=Rb_points(k_rp,:);
        %RR_points和航行遮罩一样，得到的是地图上点的横纵坐标的n*2的矩阵
        start_point(1,1) = Reach_point_now(1);
        start_point(1,2) = Reach_point_now(2);
        %TODO 测试是否需要交换坐标点
        [~, L1_paths] = FMM(FM_map,start_point',Theta_end');
        
        %重复一遍对L0的计算，只是起点变成了Reach_point
        RP_L1=[];    %针对每一点的所有theta
        for k_theta=1:1:size(Theta,1)     %针对每一个theta
            %对L0数据处理,风险积分是在栅格上求的，所有的贝叶斯推断都是在栅格上求的
            L1 = L1_paths{k_theta};
            L1=rot90(L1',2);
            %得到的是当前点针对当前Theta的L1积分值
            L1_integral=RiskIntegral(L1,FM_map);
            %得到的是当前点针对所有Theta的L1积分值的行向量
            RP_L1=[RP_L1,L1_integral];
        end
        %得到每一行代表一个可达点(RP,reach points)，每一列是一个theta的L1
        % RP_all_L1=[RP1Theta1,RP1Theta2,RP1Theta3,RP1Theta4,RP1Theta5,RP1Theta6
        %            RP2Theta1,RP2Theta2,RP2Theta3,RP2Theta4,RP2Theta5,RP2Theta6
        %            RP3Theta1,RP3Theta2,RP3Theta3,RP3Theta4,RP3Theta5,RP3Theta6
        %            RP4Theta1,RP4Theta2,RP4Theta3,RP4Theta4,RP4Theta5,RP4Theta6
        %            RP5Theta1,RP5Theta2,RP5Theta3,RP5Theta4,RP5Theta5,RP5Theta6];
        RP_all_L1=[RP_all_L1;RP_L1];
    end
    t_eq132=toc;
    disp(['      1.3绘制L1计算线积分完毕，共用时',num2str(t_eq132-t_eq131)]);
    %% 步骤1.4. 针对每一个theta生成新的公式（1）
    PrX_eq1=[];
    alpha_infer=1;
    for k_theta=1:1:size(Theta,1)
        PrX0=exp(-alpha_infer*(RP_all_L1(:,k_theta)-Theta_L0(k_theta)));
        sumK=sum(PrX0);      %ref2论文中是一个归一化常数K，用的所有PrX0_eq1的和，因此是sumK
        % 得到一个列向量，是第k_theta个theta下所有点的PrX值
        PrX=PrX0/sumK;  %可达点k_rp在k_theta下的公式（1）的值
        % 保存给下一个时刻公式（2）用
        % 每次往后补一列，这一列是第k_theta个theta下所有点的PrX值
        % 因此PrX_eq1的每一列和为1
        PrX_eq1=[PrX_eq1,PrX];
    end
    % Boat(OS).inferdata中存储的都是最近一步的数据，给下一次用
    PrRR_points=Rb_points;
    PrePrX_eq1=PrX_eq1;
    Boat(OS).InferData(TS).infer_points=PrRR_points;
    Boat(OS).InferData(TS).PrX=PrePrX_eq1;
    t_eq12=toc;
    disp(['    完成Equ1的计算，更新当前的位置分布，用时',num2str(t_eq12-t_eq11)]);
    
    %% 步骤2.公式(2)更新当前的意图分布
    pos_current(1)= round((Boat(TS).HisPos(t,1)+MapSize(1)*1852)/Res)+1;
    pos_current(2)= round((Boat(TS).HisPos(t,2)+MapSize(2)*1852)/Res)+1;
    %     row_index=ismember(PrRR_points,pos_current,'rows');
    %     row_current=find(row_index==1);
    %     if isempty(row_current)
    %         disp('错误！当前点不在上一步预测点中，程序暂停');
    %         break
    %     end
    
    % 由于精简了可达点，因此很有可能不在可达点里，但是会在这个范围中，找到最近的点即可。
    % 查找最近的点，由于路径已经栅格化，因此按照曼哈度距离计算，相同距离选第一个
    dis_row= abs(PrRR_points(:,1)-pos_current(1));
    dis_col= abs(PrRR_points(:,2)-pos_current(2));
    dis_all=dis_row+dis_col;
    [value,row_current]=min(dis_all);
    disp(['    开始Equ2的计算，当前TS位置在第',num2str(row_current),'个点，距离为',num2str(value)]);
    % 得到公式（2）右边第一个乘数Pr(Xi=xi)，是一行
    PrXTheta=PrePrX_eq1(row_current,:);  % 当前点在上一个时刻的所有theta的PrX值
    % 得到公式（2）右边第二个乘数Pr(theta0)，即上一步的公式（2）的值
    PrTheta0=Boat(OS).InferData(TS).ThetaPr0/sum(Boat(OS).InferData(TS).ThetaPr0); %PrTheta归一化，防止出现越来越小的情况
           %第一次预测，第一行是初始的针对每一个theta的先验概率
    PrTheta = PrTheta0.*PrXTheta;
    % 公式（2）更新PrTheta
    PrTheta = PrTheta/sum(PrTheta(:));
    
    % 更新当前时刻的ThetaPr，并存到Boat(OS).Theta_his中
    Boat(OS).InferData(TS).ThetaPr=PrTheta;
    Boat(OS).InferData(TS).Theta_his  =[ t,Boat(OS).InferData(TS). infer_label(end,2),PrTheta]; %存放推测历史
    
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
    RRTstart(1)=round(abs(( Boat(TS).HisPos(t,1)+MapSize(1)*1852)/Res))+1;
    RRTstart(2)=round(abs((-Boat(TS).HisPos(t,2)+MapSize(2)*1852)/Res))+1;
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
    Boat_x    = Boat(OS).HisPos(t,1);
    Boat_y    = Boat(OS).HisPos(t,2);
    Boat_theta=-Boat(OS).COG_rad; %此处为弧度制
    CAL_last  = Boat(OS).CAL_infer(TS);  %此处CAL是一个值，当前推测的OS对TS的CAL
    alpha=45;
    [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
    Boat(OS).CAL_infer(TS)=CAL_current;  %此处CAL是一个值，当前推测的OS对TS的CAL
    t_eq42=toc;
    tfor1=toc;
    disp(['    完成Equ4的计算，生成',num2str(PointCloud_N),'条MC路径，用时',num2str(t_eq42-t_eq41)]);
    disp(['    完成本轮对',num2str(TS),'船的推测，CAL为',num2str(CAL_current),'(CAL0=',num2str(CAL0(OS,TS)),'),用时',num2str(tfor1-tfor0)]);
    disp('===========================================================');
end