% ���ڸ��ݾ�����ʷ���ݲ���ÿһ�Ҵ��ľ�������
% ���Ȼ���ÿ�Ҵ��ľ���ʱ�̵�FM��ͼ�;��߽��path��������ʲô�쳣
% close all
MapSize=[8,8];
Res=18.52;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for OS=3:3
    for i=1:1:size(Boat(OS).Dechis,2)
        time=Boat(OS).Dechis(i).data(1);
        figure
        hold on
        FMmap=Boat(OS).Dechis(i).map;
        ss=pcolor(X,Y,FMmap);  %ע������Y��X���෴��
        set(ss, 'LineStyle','none');
        colorpan=ColorPanSet(0);
        colormap(colorpan);%����ɫ��
        %������ǰ�ľ���·��
        plot(Boat(OS).Dechis(i).path(:,1),Boat(OS).Dechis(i).path(:,2),'r.');
        
        for plotship=1:1:4
            %WTF:���������ĳ�ʼλ��
            ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
            %WTF:���������ĵ�ǰλ��
            ship_icon(Boat(plotship).HisPos(time,1),Boat(plotship).HisPos(time,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(time,2),plotship)
            %WTF:���������ĺ���ͼ
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
        title(['��',num2str(OS),'��',num2str(time),'ʱ�̵ĵ�',num2str(i),'�ξ��ߵ�FM��ͼ'])
        
        figure
        mesh(X,Y,FMmap);
        
        
%         figure
%         Scemap=Boat(OS).Dechis(i).Scenariomap;
%         mesh(X,Y,Scemap);
        title(['��',num2str(OS),'��',num2str(time),'ʱ�̵ĵ�',num2str(i),'�ξ��ߵĳ�����ͼ'])
    end
end
