figure
% infer_map=fliplr(count_map);
infer_map=count_map;
% 显示马赛克图,检验当前的count_map
ss=pcolor(Y,X,infer_map);  %来自pcolor的官方示例
set(ss, 'LineStyle','none');
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on
for plotship=1:1:4
    %WTF:画出船舶的初始位置
    ship_icon(Boat(plotship).HisPos(1,2),Boat(plotship).HisPos(1,3),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
    %WTF:画出船舶的结束位置
    ship_icon(Boat(plotship).HisPos(end,2),Boat(plotship).HisPos(end,3),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
    %WTF:画出过往的航迹图
    plot(Boat(plotship).HisPos(:,2),Boat(plotship).HisPos(:,3),'k.-');
end
hold on
% 显示Theta位置
plot(Theta(:,1),Theta(:,2),'r*')