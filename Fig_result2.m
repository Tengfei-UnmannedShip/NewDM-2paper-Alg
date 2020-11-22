% close all
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %ResolutionµØÍ¼µÄ·Ö±æÂÊ
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
% load('data.mat')
figure
ha = MarginEdit(2,2,[0.06  0.05],[.05  0.05],[0.05  0.05],1);
%% fig=1
fig=1
load('00101010-1.mat')
axes(ha(fig));
hold on
plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r.');
plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'go');
plot(Boat(2).goal(1),Boat(2).goal(2),'g*');
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'g.');
plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'bo');
plot(Boat(3).goal(1),Boat(3).goal(2),'b*');
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'b.');
plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k.');
plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r^');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'g^');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'b^');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k^');

hold off
grid on;
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');

xlabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
ylabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);

title(['Ê§¿Ø´¬Îª',num2str(fig),'ºÅ´¬µÄº½ÐÐ¹ì¼£'], 'FontSize',15);
box on;
clear Boat

%% fig=2
fig=2
load('10001010-1.mat')
axes(ha(fig));
hold on
plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r.');
plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'go');
plot(Boat(2).goal(1),Boat(2).goal(2),'g*');
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'g.');
plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'bo');
plot(Boat(3).goal(1),Boat(3).goal(2),'b*');
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'b.');
plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k.');
plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r^');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'g^');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'b^');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k^');

hold off
grid on;
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');

xlabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
ylabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);

title(['Ê§¿Ø´¬Îª',num2str(fig),'ºÅ´¬µÄº½ÐÐ¹ì¼£'], 'FontSize',15);
box on;
clear Boat

%% fig=3
fig=3
load('10100010.mat')
axes(ha(fig));
hold on
plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r.');
plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'go');
plot(Boat(2).goal(1),Boat(2).goal(2),'g*');
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'g.');
plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'bo');
plot(Boat(3).goal(1),Boat(3).goal(2),'b*');
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'b.');
plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k.');
plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r^');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'g^');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'b^');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k^');

hold off
grid on;
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');

xlabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
ylabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);

title(['Ê§¿Ø´¬Îª',num2str(fig),'ºÅ´¬µÄº½ÐÐ¹ì¼£'], 'FontSize',15);
box on;
clear Boat

%% fig=4
fig=4
load('10101000-1.mat')
axes(ha(fig));
hold on
plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
plot(Boat(1).HisPos(:, 1), Boat(1).HisPos(:, 2), 'r.');
plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'go');
plot(Boat(2).goal(1),Boat(2).goal(2),'g*');
plot(Boat(2).HisPos(:, 1), Boat(2).HisPos(:, 2), 'g.');
plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'bo');
plot(Boat(3).goal(1),Boat(3).goal(2),'b*');
plot(Boat(3).HisPos(:, 1), Boat(3).HisPos(:, 2), 'b.');
plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');
plot(Boat(4).HisPos(:, 1), Boat(4).HisPos(:, 2), 'k.');
plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r^');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'g^');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'b^');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k^');

hold off
grid on;
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');

xlabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
ylabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
title(['Ê§¿Ø´¬Îª',num2str(fig),'ºÅ´¬µÄº½ÐÐ¹ì¼£'], 'FontSize',15);
box on;
clear Boat



