figure
% infer_map=fliplr(count_map);
infer_map=count_map;
% ��ʾ������ͼ,���鵱ǰ��count_map
ss=pcolor(Y,X,infer_map);  %����pcolor�Ĺٷ�ʾ��
set(ss, 'LineStyle','none');
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
hold on
for plotship=1:1:4
    %WTF:���������ĳ�ʼλ��
    ship_icon(Boat(plotship).HisPos(1,2),Boat(plotship).HisPos(1,3),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
    %WTF:���������Ľ���λ��
    ship_icon(Boat(plotship).HisPos(end,2),Boat(plotship).HisPos(end,3),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
    %WTF:���������ĺ���ͼ
    plot(Boat(plotship).HisPos(:,2),Boat(plotship).HisPos(:,3),'k.-');
end
hold on
% ��ʾThetaλ��
plot(Theta(:,1),Theta(:,2),'r*')