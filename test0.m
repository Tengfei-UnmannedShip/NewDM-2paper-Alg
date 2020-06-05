
pos_last(1)=round((Boat(TS).HisPos(end-1,1)+MapSize(1)*1852)/Res)+1;
pos_last(2)=round((Boat(TS).HisPos(end-1,2)+MapSize(2)*1852)/Res)+1;

figure
hold on
plot(pos_current(1),pos_current(2),'ro')
plot(pos_last(1),pos_last(2),'bo')
plot(PrRR_points(:,1),PrRR_points(:,2),'r*')
axis([0 size(FM_map,1) 0 size(FM_map,2)])

