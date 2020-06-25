% 用于最后绘制避碰路径
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
OS=1;
ScenarioMap=zeros(m,n);
SCRMap=zeros(m,n);
CALMap=zeros(m,n);
RiskLabel=[];
Risk_temp=[];
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        
        v_os = Boat(OS).speed(end,:);
        course_os = Boat(OS).COG_deg(end,:);
        pos_os = Boat(OS).pos;
        v_ts = Boat(TS).speed(end,:);
        course_ts = Boat(TS).COG_deg(end,:);
        pos_ts = Boat(TS).pos;
        k=k+1;
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            
            %计算避碰规则下的风险场，规则场RuleField
            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
            % 这个CAL是OS对TS的CAL，为0或1
            CAL=Boat(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
            ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
            SCRMap=SCRMap+SCR_temp;
            CALMap=CALMap+CAL_Field;

    end
end


% 绘制势场图
figure;
hold on
surf(X,Y,SCRMap,'edgecolor','none','facecolor','interp') %带填充颜色的三维图
colormap jet
% kk1=mesh(X,Y,SCRMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%定义色盘

plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
axis equal
axis off

figure
hold on
kk2=contourf(X,Y,CALMap);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
% set(kk2, 'LineStyle','none');

plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');

ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );               

