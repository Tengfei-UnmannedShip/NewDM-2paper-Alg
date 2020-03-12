function L0_integral = RiskIntegral( L0,RiskMap )
% RISKINTEGRAL用于计算栅格化路径上风险的线积分

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
% 2)计算每一个格子的风险值，就是L0的风险积分
L0_integral=0;
for k_L0=1:1:size(L0_point,1)
    m_L0=L0_point(k_L0,1);
    n_L0=L0_point(k_L0,2);
    L0_integral=L0_integral+RiskMap(m_L0,n_L0);
end

end

