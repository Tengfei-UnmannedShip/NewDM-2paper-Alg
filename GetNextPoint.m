function [point,theta,index1, index2, dis1, dis2] = GetNextPoint(path, index1, index2, dis1, dis)
% path: n*2
% point: 1*2
point_pre = path(index1, :);
point_post = path(index2, :);
delta_x = point_post(1) - point_pre(1);
delta_y = point_post(2) - point_pre(2);
d = sqrt(delta_x^2 + delta_y^2);
theta = atan2d(delta_y, delta_x);  %atan2d函数算出的是-180~180的角度

d1 = dis1 + dis;

if d1 < d
    dis1 = d1;
    dis2 = d - dis1;
    point = point_pre + dis1 * [cosd(theta), sind(theta)];
else
    while(d1 >= d)
        dis1 = d1 - d;
        d1 = dis1;
        index1 = index2;
        index2 = index2 + 1;
        point_pre = path(index1, :);
        point_post = path(index2, :);
        delta_x = point_post(1) - point_pre(1);
        delta_y = point_post(2) - point_pre(2);
        d = sqrt(delta_x^2 + delta_y^2);  
        
    end
    dis2 = d - dis1;
    theta = atan2d(delta_y, delta_x);
    point = point_pre + dis1 * [cosd(theta), sind(theta)];
end
end

