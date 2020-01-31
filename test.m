Astar_map=zeros(m,n);
for i=1:1:4
    Astar_map=Astar_map+Boat(i).APF;
end

figure
kk2=contourf(X,Y,Astar_map);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
for i=1:1:4
hold on
plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat(i).State(1,1),Boat(i).State(1,2),Boat(i).State(1,5), Boat(i).State(1,6), Boat(i).State(1,3),2 );

AstarCourse=Boat(i).AsCourse_deg-180;

for ii=1:1:length(Boat(i).AsPos)
    hold on;
    ship_icon(Boat(i).AsPos(ii,1),Boat(i).AsPos(ii,2),Boat(i).State(1,5)/5, Boat(i).State(1,6)/5, AstarCourse(ii),1 );
end
end