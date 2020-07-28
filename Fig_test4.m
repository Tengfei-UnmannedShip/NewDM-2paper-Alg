% 绘制某一个时刻的船舶避碰决策路径
% 1.提取该时刻的所有船舶状态：位置、航速、航向
% 2.根据状态绘制新的地图
% 3.路径规划
tic
time=1572;   %测试的时刻
OS=2;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution地图的分辨率
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40    ];

for  i=1:4
    Ship(i).pos=Boat(i).HisPos(time,:);
    Ship(i).speed=Boat(i).speed;
    Ship(i).COG_rad=Boat(i).HisCOG(time,1);
    Ship(i).COG_deg=Boat(i).HisCOG(time,2);
    
end
Ship(OS).CAL=Boat(OS).CAL;
Ship(OS).currentWP=Boat(OS).End(393,2:3);
Ship(OS).goal=Boat(OS).goal;
Ship(OS).path=[];
ScenarioMap=zeros(m,n);

%% 目标点确定

    %    判断方法：如果已经到达场景路径点，则改为最终的目标点
%     DisWP_temp=Ship(OS).pos-Ship(OS).currentWP;
%     DisWP=norm(DisWP_temp);
%     if  DisWP<4*Res
        end_point(1,2) =round((Ship(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
        end_point(1,1) =round((Ship(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
%         disp('    已到达路径点，目标点为终点');
%     else
%         end_point(1,2) =round((Ship(OS).currentWP(1,1)+MapSize(1)*1852)/Res)+1;
%         end_point(1,1) =round((Ship(OS).currentWP(1,2)+MapSize(2)*1852)/Res)+1;
%         disp('    避碰中，目标点为场景路径点');
%     end



for TS=1:1:4
    if TS~=OS
        v_os = Ship(OS).speed(end,:);
        course_os = Ship(OS).COG_deg(end,:);
        pos_os = Ship(OS).pos;
        v_ts = Ship(TS).speed(end,:);
        course_ts = Ship(TS).COG_deg(end,:);
        pos_ts = Ship(TS).pos;
        dis_now=norm(pos_ts-pos_os);
        Dis0=2*1852;% 只有两船的距离小于2海里时，才开始绘制地图
        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA=CPA_temp(5);
        % 这样，即使目标船已经和本船背离，在FM路径规划的时候也不会把目标船考虑进去
        if  dis_now<Dis0 && DCPA<1000
            disp(['  ',num2str(OS),'号船与',num2str(TS),'号船距离为',num2str(dis_now),'需要计算目标船势场']);
            Ship(OS).decision_label=Ship(OS).decision_label+1;
            % 为了修正路径点在FM地图中的畸变，需要把路径点附近的船舶风险场变小
            DisWP_temp=pos_os-Ship(OS).currentWP;
            DisWP=norm(DisWP_temp);
            risk_factor=1;
            if  DisWP>2*Res   %未达到路径点，目标点为终点
                DisWP_temp_ts=pos_ts-Ship(OS).currentWP;
                DisWP_ts=norm(DisWP_temp_ts);
                if  DisWP_ts<=2*1852   %TS与WP的距离小于1nm时，就在TS风险范围中了，需要减小TS的影响
                    risk_factor=0.001;
                    disp(['    目标船',num2str(TS),'距离目标路径点',num2str(DisWP_ts),'修正风险场']);
                    risk_factor_count=risk_factor_count+1;
                end
            end
            Boat_theta = -Ship(TS).COG_rad(end,:); %此处为弧度制
            Boat_Speed = Ship(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            
            %计算避碰规则下的风险场，规则场RuleField
            cro_angle=abs(Ship(OS).COG_deg-Ship(TS).COG_deg);
            % 这个CAL是OS对TS的CAL，为0或1
            CAL=Ship(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
            ScenarioMap=ScenarioMap+(SCR_temp+CAL_Field)*risk_factor;
        end
    end
end
RiskMap=1./(ScenarioMap+1);
    %% 绘制当前本船的航行遮罩
    Boat_x=Ship(OS).pos(1,1);
    Boat_y=Ship(OS).pos(1,2);
    Boat_theta=-Ship(OS).COG_rad(end,:); %此处为弧度制
    % Shiplength = ShipSize(OS,1);
    alpha=30;    %30度在2*18.52的分辨率上太小了，在最后的AG_map上开口处会有一个诡异的尖刺，是由于开口附近的两个方格连在一起了
    R=500;
    AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
    [AG_row,AG_col]=find(AFMfield~=0);
    AG_points=[AG_row,AG_col];
    AG_map0=ones(size(AFMfield));
    [AG_map, ~] = FMM(AG_map0, AG_points');
    FM_map=min(AG_map,RiskMap);
    start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
    start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
    %如果start_point在FM_map的值<0.001，则FM的起始点陷入0点，无法计算路径，要加0.001
    if FM_map(start_point(1,1),start_point(1,2))<0.001
        FM_map=FM_map+0.01;
    end
    % FM_map是FM算法的输入矩阵，最大值为1，因此大于1时需要强制置为1
    FM_map(FM_map>1)=1;
    %% FM算法主程序
    
    %2. 路径规划
    %当起始点或终点到边界处时，认为在内部，没有出现过这种情况，但是懒得删了
    if start_point(1,1)>m
        start_point(1,1)=m;
    end
    if start_point(1,2)>n
        start_point(1,2)=n;
    end
    if end_point(1,1)>m
        end_point(1,1)=m;
    end
    if end_point(1,2)>n
        end_point(1,2)=n;
    end
    
    t_count21=toc;
    
    [FMoutput, paths] = FMM(FM_map,start_point',end_point');
    t_count22=toc;
    disp(['      ',num2str(OS),'号船路径规划用时: ',num2str(t_count22-t_count21)]);
   
    %3. 数据处理
    FMpath = paths{:};
    %                 FMpath=fliplr(FMpath);
    path0=rot90(FMpath',2);
    Ship(OS).AFMpath=path0;
    PathData_temp = zeros(size(path0));
    PathData=zeros(size(path0));
    PathData_temp(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    PathData_temp(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
%      %规划的路径平滑处理
%     if  size(PathData_temp,1)<400
%         Nsmooth=size(PathData_temp,1);
%     else
%         Nsmooth=400;
%     end
%     x0=PathData_temp(1:Nsmooth,1);
%     y0=PathData_temp(1:Nsmooth,2);
%     x_new=smooth(x0);
%     y_new=smooth(y0);
%     x_new1=smooth(x_new);
%     y_new1=smooth(y_new);
%     PathData_temp(1:Nsmooth,1)=x_new1;
%     PathData_temp(1:Nsmooth,2)=y_new1;
    
    
    if Ship(OS).pos(1)~=PathData_temp(1,1) || Ship(OS).pos(2)~=PathData_temp(1,2)
        % 由于栅格的存在，一般规划出的path的起点并不是决策的起点的实际位置
        % 则需要把路径平滑拟合到规划的path上
        startlines=30;
        for  i_path=1:1:startlines
            x_p0=PathData_temp(startlines+1,1);
            y_p0=PathData_temp(startlines+1,2);
            x_p1=PathData_temp(1,1);
            y_p1=PathData_temp(1,2);
            x_p2=PathData_temp(i_path,1) ;
            y_p2=PathData_temp(i_path,2);
            
            x_p10=Ship(OS).pos(1);
            y_p10=Ship(OS).pos(2);
            if  abs(x_p1-x_p0)>0.1
                lambda= (x_p2-x_p0)/(x_p1-x_p0);
            else
                lambda= (y_p2-y_p0)/(y_p1-y_p0);
            end
            x_p20=lambda*(x_p10-x_p0)+x_p0;
            y_p20=lambda*(y_p10-y_p0)+y_p0;
            
            PathData(i_path,1)=x_p20;
            PathData(i_path,2)=y_p20;
        end
        PathData(startlines+1:end,:)=PathData_temp(startlines+1:end,:);
    end
    Ship(OS).path=PathData;     %如果decision_label==0，则 Boat(OS).path保持原状
    
    %% 画图
    figure
            plot(Ship(OS).path(:,1),Ship(OS).path(:,2),'r-');
        hold on
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

