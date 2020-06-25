%用于最后绘制避碰路径
% Boat=Boat_end;
% MapSize=[8,8];
% GoalRange=MapSize-[0.75,0.75];
% Res=50;  %Resolution地图的分辨率
% [X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
% [m,n]=size(X);

%% 分时段绘图
% for i=1:1:4
%     switch i
%         case 1
%             Boat=Boat1000;
%         case 2
%             Boat=Boat1500;
%         case 3
%             Boat=Boat2000;
%         case 4
%             Boat=Boat2500;
%     end
%
%     figure(i)

% plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
% 
% plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
% 
% plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'ro');
% 
% 
% 
% plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
% 
% plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
% 
% plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'bo');
% 
% 
% 
% plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
% 
% plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
% 
% plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'go');
% 
% 
% 
% plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
% 
% plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
% 
% plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'ko');
% 
% 
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;

% end

%% 绘制单张图
figure
hold on
plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r-');
% plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-.');


plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'b-');
% plot(Boat(2).path(:, 1), Boat(2).path(:, 2), 'bo');


plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'g-');
% plot(Boat(3).path(:, 1), Boat(3).path(:, 2), 'go');


plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k-');
% plot(Boat(4).path(:, 1), Boat(4).path(:, 2), 'ko');
hold off

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;
% 
%% 绘制势场图
% figure;
% kk1=mesh(X,Y,InferMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘
% 
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% ;
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
% 
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% 
% ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );                % axis equal
% % axis off
%

