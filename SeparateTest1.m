load test-2.mat
%% 地图设置
MapSize=[8,8];
Res=18.52*2;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%% 每艘船路径规划，W1～4为每一艘船当前的FMM输入地图，start和end分别为起点和终点
FM_map=W4+0.0001;
start_point=start4;
end_point=end4;
%%注释：经测试，发现只有W1,start1,end1即第一艘船时可以正常运行，
% 其他2～4艘船虽然都是按照同样的方式生成的，但是无法运行
% 经过检查发现问题在于，船2～4时，FMM函数中第12行[D,~] = perform_fast_marching(W, start_points, options); 
% 生成的图W除了起始点的值为0，其他值全是Inf，不知道为什么？？
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
[Mtotal, paths] = FMM(FM_map,start_point',end_point');
toc

FinalMap=Mtotal;
FMpath = paths{:};
path0=fliplr(FMpath');  %FMM规划出来的路径和我的设置XY是反的
posData = zeros(size(path0));
posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;   %栅格坐标转换会实际坐标
posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;

start_pos(1)=start_point(2)*Res-MapSize(1)*1852; %FMM规划出来的路径和我的设置XY是反的
start_pos(2)=start_point(1)*Res-MapSize(2)*1852;

end_pos(1)=end_point(2)*Res-MapSize(1)*1852; %FMM规划出来的路径和我的设置XY是反的
end_pos(2)=end_point(1)*Res-MapSize(2)*1852;

figure
contourf(X,Y,FM_map);  %带填充颜色的等高线图
hold on
plot(start_pos(1,1),start_pos(1,2),'ro');
hold on
plot(end_pos(1),end_pos(2),'r*');
hold on
plot(posData(:, 1), posData(:, 2), 'r-');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

%% 输入地图对比
figure
subplot(2,2,1)
contourf(X,Y,W1*100,10);
title('1号船导引图')
subplot(2,2,2)
contourf(X,Y,W2*100,10);
title('2号船导引图')
subplot(2,2,3)
contourf(X,Y,W3*100,10);
title('3号船导引图')
subplot(2,2,4)
contourf(X,Y,W4*100,10);
title('4号船导引图')