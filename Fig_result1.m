% close all
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
% load('data.mat')
figure
ha = MarginEdit(3,2,[0.06  0.05],[.05  0.05],[0.05  0.05],1);
for fig=1:6
    
    switch fig
        case 1
            k=100;
        case 2
            k=500;
        case 3
            k=1000;
        case 4
            k=1500;
        case 5
            k=2000;
        case 6
            k=2500;
    end
axes(ha(fig));
%     hold on;
    for i=1:4
        %     WTF:画出船舶的初始位置
        drawShip(Boat(i).HisPos(1,:),Boat(i).HisCOG(1,2),i,300);
        
        %     WTF:画出船舶的结束位置
        drawShip(Boat(i).HisPos(k,:),Boat(i).HisCOG(k,2),i,300);
    end
    %     WTF:画出过往的航迹图
    plot(Boat(1).HisPos(1:k,1),Boat(1).HisPos(1:k,2),'r-');
    plot(Boat(2).HisPos(1:k,1),Boat(2).HisPos(1:k,2),'g-');
    plot(Boat(3).HisPos(1:k,1),Boat(3).HisPos(1:k,2),'b-');
    plot(Boat(4).HisPos(1:k,1),Boat(4).HisPos(1:k,2),'k-');
    
%     hold off
    grid on;
    axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
    set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    
    xlabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
    ylabel('\it n \rm miles', 'Fontname', 'Times New Roman','FontSize',13);
    
     title(['\it t \rm=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
    box on;
end

