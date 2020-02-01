%A*的栅格化结果回归实际坐标

SCR=zeros(m,n);
for i=1:1:Boat_Num
    SCR=SCR+Boat(i).SCR;
end

figure
kk2=contourf(X,Y,SCR);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
for i=2:1:2
    hold on
    plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
    hold on;
    ship_icon(Boat(i).pos(1,1),Boat(i).pos(1,2),ShipSize(i,1), ShipSize(i,2), Boat(i).COG_deg,0);
    
    for ii=1:1:length(Boat(i).AsPos)
        hold on;
        ship_icon(Boat(i).DecPos(ii,1),Boat(i).DecPos(ii,2),ShipSize(i,1), ShipSize(i,2), Boat(i).AsCourse_deg(ii),1 );
    end
end
title('valueAPF=0.1')
