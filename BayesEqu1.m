function [Rb_points,PrX_eq1] = BayesEqu1( Boat_x,Boat_y,L0_paths,Boat_theta,speed,Theta,Theta_end,FM_map,MapSize,Res)
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
t_res=ceil(Res/speed); %要保证每次至少能前进1格
%% 步骤1.2. 找出t时刻所有的可达点Reachable points(Rb_points)
% 用筛选AG_points的方法确定某一个时刻的可达点集合（r=V*(t-1),R=V*t）
Rb_points=[];
for RR_t=1:1:10   %每次猜测10步
    r_infer=0;
    R_infer=RR_t*speed*t_res;
    alpha=45;
    %RR_points和航行遮罩一样，得到的是地图上点的横纵坐标的n*2的矩阵
    RRpoint0=ReachableRange(Boat_x,Boat_y,Boat_theta,alpha,R_infer,r_infer,MapSize,Res);
    %顺着往下接这个n*2的矩阵，因此不存在尺寸不统一的问题，最后得到l*2的矩阵，l为所有3步以内可达点的个数
    Rb_points=[Rb_points;RRpoint0];
end
% 删去相同的点（即相同的行）
% 使用unique函数
Rb_points=unique(Rb_points,'rows','stable');
% unique(A,’stable’)去重复后不排序。默认的排序是unique(A,’sorted’)，’sorted’一般省略掉了。
Rb_points=fliplr(Rb_points);

%% 步骤1.3. 绘制L1计算线积分
% 对每一个RR_point找到对应的TS-Reachable-theta的路径L1
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

end

