MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52*2;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
% figure
% mesh(X,Y,W1*100);
% title('1号船导引图')
% figure
% mesh(X,Y,D1*100);
% title('1号船导引图')

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

% W11=W1;
% W11(W11>0)=10;
% W21=W2;
% W21(W21>0)=10;
% W31=W3;
% W31(W31>0)=10;
% W41=W4;
% W41(W41>0)=10;
% 
% figure
% subplot(2,2,1)
% contourf(X,Y,W11*100,10);
% title('1号船导引图')
% subplot(2,2,2)
% contourf(X,Y,W21*100,10);
% title('2号船导引图')
% subplot(2,2,3)
% contourf(X,Y,W31*100,10);
% title('3号船导引图')
% subplot(2,2,4)
% contourf(X,Y,W41*100,10);
% title('4号船导引图')
% 



% figure
% mesh(X,Y,W1);
% title('1号船导引图')
% figure
% mesh(X,Y,W2);
% title('2号船导引图')
% figure
% mesh(X,Y,W3);
% title('3号船导引图')
% figure
% mesh(X,Y,W4);
% title('4号船导引图')





% figure
% subplot(2,2,1)
% mesh(X,Y,AG_map1);
% title('1号船导引图')
% subplot(2,2,2)
% mesh(X,Y,AG_map2);
% title('2号船导引图')
% subplot(2,2,3)
% mesh(X,Y,AG_map3);
% title('3号船导引图')
% subplot(2,2,4)
% mesh(X,Y,AG_map4);
% title('4号船导引图')
% 
% figure
% subplot(2,2,1)
% mesh(X,Y,RiskMap1);
% title('1号船安全图')
% subplot(2,2,2)
% mesh(X,Y,RiskMap2);
% title('2号船安全图')
% subplot(2,2,3)
% mesh(X,Y,RiskMap3);
% title('3号船安全图')
% subplot(2,2,4)
% mesh(X,Y,RiskMap4);
% title('4号船安全图')
% 
% figure
% subplot(2,2,1)
% mesh(X,Y,FM_map1);
% title('1号船输入图')
% subplot(2,2,2)
% mesh(X,Y,FM_map2);
% title('2号船输入图')
% subplot(2,2,3)
% mesh(X,Y,FM_map3);
% title('3号船输入图')
% subplot(2,2,4)
% mesh(X,Y,FM_map4);
% title('4号船输入图')


% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
