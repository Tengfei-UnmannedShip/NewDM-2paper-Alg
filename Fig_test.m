%���������Ʊ���·��
Boat=Boat_end;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

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

% plot(Boat(1).HisPos(1,2),Boat(1).HisPos(1,3),'ro');
% 
% plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
% 
% plot(Boat(1).HisPos(:, 2), Boat(1).HisPos(:, 3), 'ro');
% 
% 
% 
% plot(Boat(2).HisPos(1,2),Boat(2).HisPos(1,3),'bo');
% 
% plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
% 
% plot(Boat(2).HisPos(:, 2), Boat(2).HisPos(:, 3), 'bo');
% 
% 
% 
% plot(Boat(3).HisPos(1,2),Boat(3).HisPos(1,3),'go');
% 
% plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
% 
% plot(Boat(3).HisPos(:, 2), Boat(3).HisPos(:, 3), 'go');
% 
% 
% 
% plot(Boat(4).HisPos(1,2),Boat(4).HisPos(1,3),'ko');
% 
% plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
% 
% plot(Boat(4).HisPos(:, 2), Boat(4).HisPos(:, 3), 'ko');
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

figure
hold on
plot(Boat(1).HisPos(1,2),Boat(1).HisPos(1,3),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 2), Boat(1).HisPos(:, 3), 'r-');
plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-.');


plot(Boat(2).HisPos(1,2),Boat(2).HisPos(1,3),'bo');
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
plot(Boat(2).HisPos(:, 2), Boat(2).HisPos(:, 3), 'b-');

% plot(Boat(2).path(:, 1), Boat(2).path(:, 2), 'bo');
% 

plot(Boat(3).HisPos(1,2),Boat(3).HisPos(1,3),'go');
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
plot(Boat(3).HisPos(:, 2), Boat(3).HisPos(:, 3), 'g-');

% plot(Boat(3).path(:, 1), Boat(3).path(:, 2), 'go');
% 

plot(Boat(4).HisPos(1,2),Boat(4).HisPos(1,3),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 2), Boat(4).HisPos(:, 3), 'k-');
hold off
% plot(Boat(4).path(:, 1), Boat(4).path(:, 2), 'ko');


% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
% 

% figure;
% kk1=mesh(X,Y,ScenarioMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
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
% kk2=contourf(X,Y,ScenarioMap);  %�������ɫ�ĵȸ���ͼ
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% % set(kk2, 'LineStyle','none');
% 
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% 
% ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );                % axis equal
% % axis off
%

