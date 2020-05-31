function L0_integral = RiskIntegral( L0,RiskMap )
% RISKINTEGRAL用于计算栅格化路径上风险的线积分
%规划的路径平滑处理
% 防止出现剧烈拐点的地方，顺次的计算将会不可实现，如果没有拐点，平滑将不改变原曲线
% 由于这种情况一般只会发生在路径开始的部分，因此只做前50个点的路径平滑
x0=L0(1:50,1); 
y0=L0(1:50,2);
% 平滑两次
y_new=smooth(y0);
x_new=smooth(x0);
x_new1=smooth(x_new);
y_new1=smooth(y_new);

%得出最终的平滑后的L0
L0(1:50,1)=x_new1;
L0(1:50,2)=y_new1; 


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

