%用于绘制马赛克图
figure
%                     infer_map=zeros(m,n);
%                     for ts_map=1:1:Boat_Num
%                         if ts_map~=OS &&  ismember(t,Boat(OS).InferData(ts_map).infer_label)
%                             infer_map=infer_map+Boat(OS).InferData(ts_map).infermap(t).map;
%                         end
%                     end

infer_map0=Boat(1).InferData(2).infermap(300).map;
infer_map0(infer_map0>5)=5;

% 显示马赛克图,检验当前的count_map
ss=pcolor(Y,X,infer_map0);  %注意这里Y，X是相反的
set(ss, 'LineStyle','none');
colorpan=ColorPanSet(0);
colormap(colorpan);%定义色盘
hold on
for plotship=1:1:4
    %WTF:画出船舶的初始位置
    ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
    %WTF:画出船舶的结束位置
    ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
    %WTF:画出过往的航迹图
    plot(Boat(plotship).HisPos(:,1),Boat(plotship).HisPos(:,2),'k.-');
end
hold on
% 显示Theta位置
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