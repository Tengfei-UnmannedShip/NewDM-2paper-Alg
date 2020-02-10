for i=1:1:size(Boat(1).AsPos)
    ship_icon(Boat(1).AsPos(i,1),Boat(1).AsPos(i,2),ShipSize(1,1),ShipSize(1,2),Boat(1).AsPos(i,2),1)
    hold on
end
for i=1:1:size(Boat(2).AsPos)
    ship_icon(Boat(2).AsPos(i,1),Boat(2).AsPos(i,2),ShipSize(2,1),ShipSize(2,2),Boat(2).AsPos(i,2),2)
    hold on
end
for i=1:4
    plot(Start_pos(i,1),Start_pos(i,2),'ko')
    hold on
    plot(Boat(i).goal(1),Boat(i).goal(2),'k*')
    hold on
end