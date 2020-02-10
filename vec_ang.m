function theta = vec_ang(pos,wp,course )
% 计算当前本船的路径点是在本船前方还是后方
% 输入：
%    1. 本船当前坐标
%    2. 本船当前的路径点
%    3. 本船当前的航向角，角度制，正北为0
% 输出：路径点在本船当前的方向角（类似几点钟方向）

a = wp-pos;     %以路径点为顶点，以当前位置为起点的向量
b = [sind(course) cosd(course)]; %本船当前前进的方向向量
costheta=dot(a,b)/(norm(a)*norm(b));
%注意，计算出的夹角，只有角度（0～180），没有方向
theta = acosd(costheta);
end