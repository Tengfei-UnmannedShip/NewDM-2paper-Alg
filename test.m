figure
kk2=contourf(X,Y,Astar_map);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on
plot(Boat(1).goal(1,1),Boat(1).goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat(1).State(1,1),Boat(1).State(1,2),Boat(1).State(1,5), Boat(1).State(1,6), Boat(1).State(1,3),2 );

AstarCourse=Boat(1).Astar_course-180;

for i=1:1:length(Boat(1).Astar_pos)
    hold on;
    ship_icon(Boat(1).Astar_pos(i,1),Boat(1).Astar_pos(i,2),Boat(1).State(1,5)/5, Boat(1).State(1,6)/5, AstarCourse(i),1 );
end