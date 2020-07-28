% ����ĳһ��ʱ�̵Ĵ�����������·��
% 1.��ȡ��ʱ�̵����д���״̬��λ�á����١�����
% 2.����״̬�����µĵ�ͼ
% 3.·���滮
tic
time=1572;   %���Ե�ʱ��
OS=2;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution��ͼ�ķֱ���
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

%% Ŀ���ȷ��

    %    �жϷ���������Ѿ����ﳡ��·���㣬���Ϊ���յ�Ŀ���
%     DisWP_temp=Ship(OS).pos-Ship(OS).currentWP;
%     DisWP=norm(DisWP_temp);
%     if  DisWP<4*Res
        end_point(1,2) =round((Ship(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
        end_point(1,1) =round((Ship(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
%         disp('    �ѵ���·���㣬Ŀ���Ϊ�յ�');
%     else
%         end_point(1,2) =round((Ship(OS).currentWP(1,1)+MapSize(1)*1852)/Res)+1;
%         end_point(1,1) =round((Ship(OS).currentWP(1,2)+MapSize(2)*1852)/Res)+1;
%         disp('    �����У�Ŀ���Ϊ����·����');
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
        Dis0=2*1852;% ֻ�������ľ���С��2����ʱ���ſ�ʼ���Ƶ�ͼ
        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA=CPA_temp(5);
        % ��������ʹĿ�괬�Ѿ��ͱ������룬��FM·���滮��ʱ��Ҳ�����Ŀ�괬���ǽ�ȥ
        if  dis_now<Dis0 && DCPA<1000
            disp(['  ',num2str(OS),'�Ŵ���',num2str(TS),'�Ŵ�����Ϊ',num2str(dis_now),'��Ҫ����Ŀ�괬�Ƴ�']);
            Ship(OS).decision_label=Ship(OS).decision_label+1;
            % Ϊ������·������FM��ͼ�еĻ��䣬��Ҫ��·���㸽���Ĵ������ճ���С
            DisWP_temp=pos_os-Ship(OS).currentWP;
            DisWP=norm(DisWP_temp);
            risk_factor=1;
            if  DisWP>2*Res   %δ�ﵽ·���㣬Ŀ���Ϊ�յ�
                DisWP_temp_ts=pos_ts-Ship(OS).currentWP;
                DisWP_ts=norm(DisWP_temp_ts);
                if  DisWP_ts<=2*1852   %TS��WP�ľ���С��1nmʱ������TS���շ�Χ���ˣ���Ҫ��СTS��Ӱ��
                    risk_factor=0.001;
                    disp(['    Ŀ�괬',num2str(TS),'����Ŀ��·����',num2str(DisWP_ts),'�������ճ�']);
                    risk_factor_count=risk_factor_count+1;
                end
            end
            Boat_theta = -Ship(TS).COG_rad(end,:); %�˴�Ϊ������
            Boat_Speed = Ship(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            
            %������������µķ��ճ�������RuleField
            cro_angle=abs(Ship(OS).COG_deg-Ship(TS).COG_deg);
            % ���CAL��OS��TS��CAL��Ϊ0��1
            CAL=Ship(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
            ScenarioMap=ScenarioMap+(SCR_temp+CAL_Field)*risk_factor;
        end
    end
end
RiskMap=1./(ScenarioMap+1);
    %% ���Ƶ�ǰ�����ĺ�������
    Boat_x=Ship(OS).pos(1,1);
    Boat_y=Ship(OS).pos(1,2);
    Boat_theta=-Ship(OS).COG_rad(end,:); %�˴�Ϊ������
    % Shiplength = ShipSize(OS,1);
    alpha=30;    %30����2*18.52�ķֱ�����̫С�ˣ�������AG_map�Ͽ��ڴ�����һ������ļ�̣������ڿ��ڸ�����������������һ����
    R=500;
    AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
    [AG_row,AG_col]=find(AFMfield~=0);
    AG_points=[AG_row,AG_col];
    AG_map0=ones(size(AFMfield));
    [AG_map, ~] = FMM(AG_map0, AG_points');
    FM_map=min(AG_map,RiskMap);
    start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
    start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
    %���start_point��FM_map��ֵ<0.001����FM����ʼ������0�㣬�޷�����·����Ҫ��0.001
    if FM_map(start_point(1,1),start_point(1,2))<0.001
        FM_map=FM_map+0.01;
    end
    % FM_map��FM�㷨������������ֵΪ1����˴���1ʱ��Ҫǿ����Ϊ1
    FM_map(FM_map>1)=1;
    %% FM�㷨������
    
    %2. ·���滮
    %����ʼ����յ㵽�߽紦ʱ����Ϊ���ڲ���û�г��ֹ������������������ɾ��
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
    disp(['      ',num2str(OS),'�Ŵ�·���滮��ʱ: ',num2str(t_count22-t_count21)]);
   
    %3. ���ݴ���
    FMpath = paths{:};
    %                 FMpath=fliplr(FMpath);
    path0=rot90(FMpath',2);
    Ship(OS).AFMpath=path0;
    PathData_temp = zeros(size(path0));
    PathData=zeros(size(path0));
    PathData_temp(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    PathData_temp(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
%      %�滮��·��ƽ������
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
        % ����դ��Ĵ��ڣ�һ��滮����path����㲢���Ǿ��ߵ�����ʵ��λ��
        % ����Ҫ��·��ƽ����ϵ��滮��path��
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
    Ship(OS).path=PathData;     %���decision_label==0���� Boat(OS).path����ԭ״
    
    %% ��ͼ
    figure
            plot(Ship(OS).path(:,1),Ship(OS).path(:,2),'r-');
        hold on
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

