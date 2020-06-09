% С�������ռ���OccupancyGrid��ʾ����ʵ����դ��
% ������robotics.BinaryOccupancyGrid��ʾ(BinaryOG):1��ʾ�ϰ��0��ʾ���ߡ�
% ������map = robotics.BinaryOccupancyGrid(10,10,5)��������һ��BOG��10�׳�10�ף�ÿ�ױ��ֳ�5�ȷݣ��ֱ���Ϊ5��,
% ��setOccupancy(map,�ϰ�������,1)������map����ϰ��
% ������robotics.OccupancyGrid��ʾ(ProbabilityOccupancyGrid)��1�ܿ������ϰ��0ûɶ�������ϰ���
% ���������յ㣬������PRM��prob road map���㷨����һ��û���ϰ����·���������������ƽ���������һЩ�㣬
% Ȼ������е����������������ϰ��������������յ�����������֮���·������Ҫ��·����������PRMobj = robotics.PRM(map,250)��250���㵽map�ϣ�������PRM��
% PRMobj.show���Ի���������PRMobj.findpath([�������],[Ŀ�������])�����ҵ�һ����㵽�յ�����߲�����������ÿ��������꣬���ʱ����PRMobj.show���Կ��������ߡ�
% ��ʱ������˱Ƚϴ󣬹滮������·�߿��ܿ��ϰ���̫���ˣ���ʱ������Ȱ�map����ϰ�������һ�㣬�ٹ滮·����

map0=PRM_map;

map0(map0<40)=0;
map0(map0>=40)=1;

map0=flipud(map0);
% imshow(map0)


% figure
% for plotship=1:1:4
%     %WTF:���������Ľ���λ��
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
disp(['����',num2str(point_num),'�����PRM��ͼ��ʱ',num2str(t_map1-t_map0)]);
% figure
% show(prmSimple)
t_path0=toc;

startLocation(1,2) = round((Boat(2).pos(1)+MapSize(1)*1852)/Res)+1;
startLocation(1,1) = round((Boat(2).pos(2)+MapSize(2)*1852)/Res)+1;

endLocation(1,2) = round((Boat(2).goal(1)+MapSize(1)*1852)/Res)+1;
endLocation(1,1) = round((Boat(2).goal(2)+MapSize(1)*1852)/Res)+1;
path =findpath(prmSimple,startLocation,endLocation);
t_path1=toc;
disp(['����·����ʱ',num2str(t_path1-t_path0)]);
toc
figure
show(prmSimple)