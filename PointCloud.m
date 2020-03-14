function Pos_pre = PointCloud( core,n,En,He ) 
%% 点云生成程序，生成围绕某一个目标位置点的服从2维正态分布的点云
% core 为核心位置
% n为生成的点数；
% En=0.005;     %控制随机偏离，一次线性，熵
% He=0.1;       %控制随机偏离，二次线性，超熵
% n=1000;
x=zeros(1,n);
y=zeros(1,n);
Ex=core(1);     %x核心位置
Ey=core(2);     %y核心位置
for i=1:n    %最外围生成每个点
    Ennx=randn(1)*He+En;
    x(i)=randn(1)*Ennx+Ex;
    Enny=randn(1)*He+En;
    y(i)=randn(1)*Enny+Ey;
end
Pos_pre=[x;y];

end

