%用于绘制船舶路径点
figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end
% 场景1，符合避碰规则时
scen=1;
plot(Boat(1).waypoint(scen,1),Boat(1).waypoint(scen,2),'r*');
plot(Boat(1).centre(scen,1),Boat(1).centre(scen,2),'ro');
plot(Boat(2).waypoint(scen,1),Boat(2).waypoint(scen,2),'b*');
plot(Boat(2).centre(scen,1),Boat(2).centre(scen,2),'bo');
plot(Boat(3).waypoint(scen,1),Boat(3).waypoint(scen,2),'g*');
plot(Boat(3).centre(scen,1),Boat(3).centre(scen,2),'go');
plot(Boat(4).waypoint(scen,1),Boat(4).waypoint(scen,2),'k*');
plot(Boat(4).centre(scen,1),Boat(4).centre(scen,2),'ko');
% 场景2，不符合避碰规则时
scen=2;
plot(Boat(1).waypoint(scen,1),Boat(1).waypoint(scen,2),'r^');
plot(Boat(2).waypoint(scen,1),Boat(2).waypoint(scen,2),'b^');
plot(Boat(3).waypoint(scen,1),Boat(3).waypoint(scen,2),'g^');
plot(Boat(4).waypoint(scen,1),Boat(4).waypoint(scen,2),'k^');

plot(Boat(1).centre(scen,1),Boat(1).centre(scen,2),'rs');
plot(Boat(2).centre(scen,1),Boat(2).centre(scen,2),'bs');
plot(Boat(3).centre(scen,1),Boat(3).centre(scen,2),'gs');
plot(Boat(4).centre(scen,1),Boat(4).centre(scen,2),'ks');
% 场景3，失控船时
scen=3;
plot(Boat(1).waypoint(scen,1),Boat(1).waypoint(scen,2),'rp');
plot(Boat(2).waypoint(scen,1),Boat(2).waypoint(scen,2),'bp');
plot(Boat(3).waypoint(scen,1),Boat(3).waypoint(scen,2),'gp');
plot(Boat(4).waypoint(scen,1),Boat(4).waypoint(scen,2),'kp');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;

%画出当前场景

figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end

plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r*');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'b*');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'g*');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;

