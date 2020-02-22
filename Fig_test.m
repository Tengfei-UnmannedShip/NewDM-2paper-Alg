figure

plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
hold on
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
hold on
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r-');
hold on

plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
hold on
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
hold on
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'b-');
hold on

plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
hold on
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
hold on
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'g-');
hold on

plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
hold on
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
hold on
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k-');
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;

% figure;
% kk1=mesh(X,Y,ScenarioMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% hold on
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% hold on;
% ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
% axis equal
% axis off
% %     surf(X,Y,APFValue);
% 
% figure
% % kk2=pcolor(APFValue);
% kk2=contourf(X,Y,ScenarioMap);  %带填充颜色的等高线图
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% % set(kk2, 'LineStyle','none');
% hold on
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% hold on
% ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );                % axis equal
% % axis off
% 

