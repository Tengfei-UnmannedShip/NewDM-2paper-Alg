% figure
% 
% [M1,c] = contourf(X,Y,M);
% c.LineWidth = 0.001;
% colorpan=ColorPanSet(6);
% colormap(colorpan);%∂®“Â…´≈Ã
% hold on
% plot(Boat(1).path(:, 1), Boat(1).path(:,2), '-w');
% 
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
% box on;

figure

plot(Boat(1).pos(1),Boat(1).pos(2),'ro');
hold on
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
hold on
plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-');
hold on

plot(Boat(2).pos(1),Boat(2).pos(2),'bo');
hold on
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
hold on
plot(Boat(2).path(:, 1), Boat(2).path(:, 2), 'b-');
hold on

plot(Boat(3).pos(1),Boat(3).pos(2),'go');
hold on
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
hold on
plot(Boat(3).path(:, 1), Boat(3).path(:, 2), 'g-');
hold on

plot(Boat(4).pos(1),Boat(4).pos(2),'ko');
hold on
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
hold on
plot(Boat(4).path(:, 1), Boat(4).path(:, 2), 'k-');

