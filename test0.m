figure
hold on
plot(pos_current(1),pos_current(2),'bo')

plot(PrRR_points(1,1),PrRR_points(1,2),'k*')
plot(PrRR_points(2:end,1),PrRR_points(2:end,2),'r*')
grid on
axis([0 size(FM_map,1) 0 size(FM_map,2)])

