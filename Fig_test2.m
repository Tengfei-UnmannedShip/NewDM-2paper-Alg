%用于绘制马赛克图
figure
infer_map=count_map;
% 显示马赛克图,检验当前的count_map
ss=pcolor(Y,X,infer_map);  %注意这里Y，X是相反的
set(ss, 'LineStyle','none');
colorpan=ColorPanSet(0);
colormap(colorpan);%定义色盘
hold on
for plotship=1:1:4
    %WTF:画出船舶的初始位置
    ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
    %WTF:画出船舶的结束位置
    ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
    %WTF:画出过往的航迹图
    plot(Boat(plotship).HisPos(:,1),Boat(plotship).HisPos(:,2),'k.-');
end
hold on
% 显示Theta位置
plot(Theta(:,1),Theta(:,2),'r*')
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;