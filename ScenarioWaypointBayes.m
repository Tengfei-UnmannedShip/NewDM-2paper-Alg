function [WayPoint_OS,C] = ScenarioWaypointBayes(W, W0,alpha_centre)
%% 当前场景的3艘目标船的综合路径点计算
% 输入：
% 当前的风险参数：R(Risk_value)
% 当前的三个路径点：W(WayPoint_temp)
% 输出：最终得到的综合路径点
% 计算思路：
%     1.找到距离三个点相等的点，即三角形的内心
%     2.找到当前的首要避让船（右侧的第一艘船）的路径点和内心的中点，即为路径点
%% 1.计算三角形的内心，使用circumcenter函数
% 三角形外接圆圆心是三边垂直平分线的交点
% 任意选两边,分别就出他们垂直平分线的方程
% 联立成为二元一次方程组就可以了解得外心坐标了
%设三点为
A1=W(1,:); 
A2=W(2,:);
A3=W(3,:);
%则A1A2的垂直平分线方程为 （x1-x2）x + （y1-y2）y = [(x1^2-x2^2)+(y1^2-y^2)]/2
% A2A3的垂直平分线方程为 （x2-x3）x + （y2-y3）y = [(x2^2-x3^2)+(y2^2-y3^2)]/2
% 写成矩阵就是
% | （x1-x2） （y1-y2）| x [(x1^2-x2^2)+(y1^2-y2^2)]/2
% | | * =
% | (x2-x3） （y2-y3）| y [(x2^2-x3^2)+(y2^2-y3^2)]/2

%用matlab的\就可以解出来了
A=[A1-A2;A2-A3];
B=([sum(A1.^2-A2.^2); sum(A2.^2-A3.^2)])/2;
if det(A)~=0
    C=(A\B)';%O=[x y]是圆心坐标
%     r=sqrt(sum((O-A1).^2));%三点找一点算半径
else
    error('不是三角形');  % 三点共线,不形成三角形就无解
end
x=[];  
y=[];

%%  2.找到当前的首要避让船（右侧的第一艘船）的路径点和内心的alpha_centre，即为路径点
for i=1:size(W0,2)
    if W0(i)==1
        x=alpha_centre*W(i,1)+(1-alpha_centre)*C(1);
        y=alpha_centre*W(i,2)+(1-alpha_centre)*C(2);
    end
end

WayPoint_OS=[x  y];


end

