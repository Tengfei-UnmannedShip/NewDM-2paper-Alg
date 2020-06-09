% 小车工作空间用OccupancyGrid表示，其实就是栅格
% 可以用robotics.BinaryOccupancyGrid表示(BinaryOG):1表示障碍物，0表示能走。
% 可以用map = robotics.BinaryOccupancyGrid(10,10,5)命令生成一个BOG，10米乘10米，每米被分成5等份（分辨率为5）,
% 用setOccupancy(map,障碍物坐标,1)可以往map里加障碍物。
% 可以用robotics.OccupancyGrid表示(ProbabilityOccupancyGrid)：1很可能有障碍物，0没啥可能有障碍物
% 给定起点和终点，可以用PRM（prob road map）算法来找一条没有障碍物的路，其基本流程是在平面上随机撒一些点，
% 然后把所有点连起来（不经过障碍物），最后离起点和终点最近的随机点之间的路径就是要的路径。可以用PRMobj = robotics.PRM(map,250)撒250个点到map上，并生成PRM。
% PRMobj.show可以画出来看，PRMobj.findpath([起点坐标],[目标点坐标])可以找到一条起点到终点的折线并返回折线上每个点的坐标，这个时候再PRMobj.show可以看到这条线。
% 有时候机器人比较大，规划出来的路线可能靠障碍物太近了，这时候可以先把map里的障碍物膨胀一点，再规划路径。

map0=PRM_map;

map0(map0<40)=0;
map0(map0>=40)=1;

map0=flipud(map0);
% imshow(map0)


% figure
% for plotship=1:1:4
%     %WTF:画出船舶的结束位置
%     ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
% end
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
% startup_rvc
% map = binaryOccupancyMap(map0,2);
% map = robotics.BinaryOccupancyGrid(100,100) ;
tic
t_map0=toc;
point_num=1000;
map = robotics.BinaryOccupancyGrid(map0) ;
prmSimple = robotics.PRM(map,point_num);
t_map1=toc;
disp(['生成',num2str(point_num),'个点的PRM地图用时',num2str(t_map1-t_map0)]);
% figure
% show(prmSimple)
t_path0=toc;

startLocation(1,2) = round((Boat(2).pos(1)+MapSize(1)*1852)/Res)+1;
startLocation(1,1) = round((Boat(2).pos(2)+MapSize(2)*1852)/Res)+1;

endLocation(1,2) = round((Boat(2).goal(1)+MapSize(1)*1852)/Res)+1;
endLocation(1,1) = round((Boat(2).goal(2)+MapSize(1)*1852)/Res)+1;
path =findpath(prmSimple,startLocation,endLocation);
t_path1=toc;
disp(['生成路径用时',num2str(t_path1-t_path0)]);
toc
figure
show(prmSimple)