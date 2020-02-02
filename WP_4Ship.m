function x=WP_4Ship(Dis,waypoint)
%% 4艘船（OS和3艘TS）的路径点计算
% 一种典型的拟牛顿法broyden计算非线性方程组的解
% 程序参考《MATLAB数值分析与应用_宋叶志》
% 实验 6.5 拟牛顿法（Broyden 方法）（全书第248页）
% 输入:
% % 位置信息pos，用于计算到各路径点的加权平均值: 
% % 本船位置
% OS_pos=pos(1,:);
% % 其他目标船位置
% TS_pos=pos(2:end,:);
% x0 为迭代初值
pos0=[0,0];
% tol 为误差容限，如果缺省默认为10的-10次方
tol=1e-5;
% data 用来存放计算的中间数据便于计算收敛情况分析

H0 = Jacobi4WP(pos0,Dis,waypoint);
H0 = inv(H0);
x1=pos0-(H0*Fun_WP(pos0,Dis,waypoint))' ;

n=1;

%设置初始误差使之可以进入循环
wucha=0.1;
%循环迭代
while (wucha>tol) && (n<200)
    
    wucha=norm(x1-pos0);
    dx=x1-pos0;
    y=Fun_WP(x1,Dis,waypoint)-Fun_WP(pos0,Dis,waypoint);
    fenzi=dx*H0*y; %是标量
    H1=H0+(dx'-H0*y)*dx*H0/fenzi;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %这一段相当重要，技巧性也比较强，请体会
    temp_x0=pos0;
    pos0=x1;
    x1=temp_x0-(H1*Fun_WP(temp_x0,Dis,waypoint))'; %x1 的更新
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %更新 H 矩阵
    H=H1;
    n=n+1;
    %data 用来存放中间数据
    data(:,n)=x1;
    
end
x=x1;
end

