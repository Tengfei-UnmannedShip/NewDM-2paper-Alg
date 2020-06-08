%���ڻ���������ͼ
figure
infer_map=count_map;
% ��ʾ������ͼ,���鵱ǰ��count_map
ss=pcolor(Y,X,infer_map);  %ע������Y��X���෴��
set(ss, 'LineStyle','none');
colorpan=ColorPanSet(0);
colormap(colorpan);%����ɫ��
hold on
for plotship=1:1:4
    %WTF:���������ĳ�ʼλ��
    ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
    %WTF:���������Ľ���λ��
    ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
    %WTF:���������ĺ���ͼ
    plot(Boat(plotship).HisPos(:,1),Boat(plotship).HisPos(:,2),'k.-');
end
hold on
% ��ʾThetaλ��
plot(Theta(:,1),Theta(:,2),'r*')
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;