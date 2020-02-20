function newPoint = WP_regiontest(WP0,MapSize,pos)
% 计算完路径点后，判断当前的路径点是否在地图范围内，不在的话，新路径点为原路径点与本船位置连线与边界的交点
% 共有三种情况，x超了，y超了，xy都超了，
region=(MapSize-[0.25,0.25])*1852;
if abs(WP0(1))<= region(1) && abs(WP0(2))<= region(2)  %均在范围内
    newPoint = WP0;   %路径点不变
else %超出范围之后，实际上就是找从当前位置出发，到路径点的在边界上的点，可以直接引用Goal_point函数
    %首先找到旧路径点与本船位置与y轴正方向夹角
    WP_pos=WP0-pos;
    y=[0,1];
    ang0=acos(dot(WP_pos,y)/(norm(WP_pos)*norm(y)));
    if WP_pos(1)>0
        ang=ang0;
    else
        ang=-ang0;
    end
    %按照Goal_point算法算出的新的路径点
    newPoint = Goal_point(pos(1),pos(2),ang,region);
    
end
end

