if isempty(Real_CAL)
    Boat(OS).WayPoint=Boat(OS).goal;  %没有路径点的时候，直接用终点作为目标点
    disp([num2str(OS),'船目标点为终点']);
else
    CAL_row=[];
    switch WP_Num
        case 1
            CAL_row=Real_CAL+1;
            disp([num2str(OS),'船有1艘风险船，场景为',num2str(Real_CAL)]);
        case 2
            CAL_row=2^Real_CAL(1)+Real_CAL(2)+1;
            disp([num2str(OS),'船有2艘风险船，场景为',num2str(Real_CAL)]);
        case 3
            CAL_row=4^Real_CAL(1)+2^Real_CAL(2)+Real_CAL(3)+1;
            disp([num2str(OS),'船有3艘风险船，场景为',num2str(Real_CAL)]);
    end
    % 判断当前的路径点是否在地图范围内，不在的话，新路径点为原路径点与本船位置连线与边界的交点
    WP_test0=Boat(OS).WayPoint_temp(CAL_row,:);
    Boat(OS).WayPoint=WP_regiontest(WP_test0,MapSize,Boat(OS).pos);
end

