% 用于根据决策历史数据测试每一艘船的决策内容
% 首先画出每艘船的决策时刻的FM地图和决策结果path，看看有什么异常
% close all
MapSize=[8,8];
Res=18.52;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for OS=3:3
    for i=1:1:size(Boat(OS).Dechis,2)
        time=Boat(OS).Dechis(i).data(1);
        figure
        hold on
        FMmap=Boat(OS).Dechis(i).map;
        ss=pcolor(X,Y,FMmap);  %注意这里Y，X是相反的
        set(ss, 'LineStyle','none');
        colorpan=ColorPanSet(0);
        colormap(colorpan);%定义色盘
        %画出当前的决策路径
        plot(Boat(OS).Dechis(i).path(:,1),Boat(OS).Dechis(i).path(:,2),'r.');
        
        for plotship=1:1:4
            %WTF:画出船舶的初始位置
            ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
            %WTF:画出船舶的当前位置
            ship_icon(Boat(plotship).HisPos(time,1),Boat(plotship).HisPos(time,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(time,2),plotship)
            %WTF:画出过往的航迹图
            plot(Boat(plotship).HisPos(1:time,1),Boat(plotship).HisPos(1:time,2),'k.-');
        end
        
        axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
        set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
        set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
        set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
        set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
        grid on;
        xlabel('\it n miles', 'Fontname', 'Times New Roman');
        ylabel('\it n miles', 'Fontname', 'Times New Roman');
        box on;
        title(['船',num2str(OS),'在',num2str(time),'时刻的第',num2str(i),'次决策的FM地图'])
        
        figure
        mesh(X,Y,FMmap);
        
        
%         figure
%         Scemap=Boat(OS).Dechis(i).Scenariomap;
%         mesh(X,Y,Scemap);
        title(['船',num2str(OS),'在',num2str(time),'时刻的第',num2str(i),'次决策的场景地图'])
    end
end
