function [RR_points,PrX_eq1] = BayesEqu1( Boat_x,Boat_y,L0_paths,Boat_theta,speed,Theta,Theta_end,FM_map,MapSize,Res)
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
    L0=rot90(L0',2);
    L0_integral=RiskIntegral(L0,FM_map);
    
    Theta_L0=[Theta_L0;L0_integral];
end

t_res=ceil(Res/speed); %要保证每次至少能前进1格

%% 步骤1.2. 找出t时刻所有的可达点Reachable
% 用筛选AG_points的方法确定某一个时刻的可达点集合（r=V*(t-1),R=V*t）
r_infer=0;
R_infer=speed*t_res;
alpha=30;
%RR_points和航行遮罩一样，得到的是地图上点的横纵坐标的n*2的矩阵
RR_points=ReachableRange(Boat_x,Boat_y,Boat_theta,alpha,R_infer,r_infer,MapSize,Res);

%% 步骤1.3. 绘制L1计算线积分
% 对每一个RR_point找到对应的TS-Reachable-theta的路径L1
Reachs_L1=[];
for k_reach=1:1:size(RR_points,1)
    Reach_point=RR_points(k_reach,:);
    %RR_points和航行遮罩一样，得到的是地图上点的横纵坐标的n*2的矩阵
    start_point(1,2) = Reach_point(1);
    start_point(1,1) = Reach_point(2);
    [~, L1_paths] = FMM(FM_map,start_point',Theta_end');
    %重复一遍对L0的计算，只是起点变成了Reach_point
    Reach_L1=[];    %针对每一点的所有theta
    Reachs_L1=[];   %针对所有点的所有theta
    for k_theta=1:1:size(Theta,1)     %针对每一个theta
        %对L0数据处理,风险积分是在栅格上求的，所有的贝叶斯推断都是在栅格上求的
        L1 = L1_paths{k_theta};
        L1=rot90(L1',2);
        %得到的是当前点针对当前Theta的L1积分值
        L1_integral=RiskIntegral(L1,FM_map);
        %得到的是当前点针对所有Theta的L1积分值的行向量
        Reach_L1=[Reach_L1,L1_integral];
    end
    %得到每一行代表一个可达点，每一列是一个theta的L1
    Reachs_L1=[Reachs_L1;Reach_L1];
end

%% 步骤1.4. 针对每一个theta生成新的公式（1）
PrX_eq1=[];
PrX0_eq1=[];
alpha_infer=1;
for k_theta=1:1:size(Theta,1)
    for k_reach=1:1:size(RR_points,1)
        PrX0=-alpha_infer*(Reachs_L1(k_reach,k_theta)-Theta_L0(k_theta));
        PrX0=exp(PrX0);
        PrX0_eq1=[PrX0_eq1,PrX0];   %每一次往后补一个数，这个数是第k_reach个可达点的PrX0
    end
    PrX_temp=[];
    for k_reach=1:1:size(RR_points,1)
        PrX=PrX0_eq1(k_reach)/sum(PrX0_eq1);  %可达点k_reach在k_theta下的公式（1）的值
        % 得到一个列向量，是第k_theta个theta下所有点的PrX值
        PrX_temp=[PrX_temp;PrX];   %每次往下补一数，值为第k_reach个可达点在第k_theta个theta下的PrX值
    end
    %保存给下一个时刻公式（2）用
    PrX_eq1=[PrX_eq1,PrX_temp];    %每次往后补一列，这一列是第k_theta个theta下所有点的PrX值
end

end

