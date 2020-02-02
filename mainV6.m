%% (3.0�汾)���·����
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.A*��APF��ϵĵ����棬A*��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
%   (2.5�汾)��WangNing��ShipDomain����ԭAPF����չ������Ϊ��λ�ĵ�ͼ
% 2.(3.0�汾)���·����
% 3.(4.0�汾)��ӱ�Ҷ˹

clear
clc
close all
tic;%tic1
%% ��ʼ����
MapSize=[8,8];
GoalRange=MapSize-[1,1];
Res=100;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

% =========================================================================
% ������������
% =========================================================================
% 1.compliance:�������������;2.inferLabbel:�Ƿ��Ʋ�
shipLabel=[
    0 0
    1 0
    1 0
    1 0];
%1~2λ��(�м��λ�ã�������ʼλ��)��3����(��)��4��ʼ����deg������Ϊ0����5��������ʱ����6��ⷶΧ��range��nm��
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40
    ];
%��ʼ���������д���ȫ������
CAL=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

Boat_Num=4;   %��������
tMax=2500;    %���ʱ��
tBayes=2000;  %��Ҷ˹�Ʋ�ʱ��

for i=1:1:Boat_Num
    
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*0.5*tMax, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*0.5*tMax];
    %     Boat(i).pos_nm=Boat(i).pos/1852;    %��λΪ�����pos
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    %     Boat(i).goal_nm=Boat(i).goal/1852;  %��λΪ�����Ŀ��λ��
    Boat(OS).WP_data=[];
end

for t=1:1:tMax
    %% ÿ��ʱ�̵�״̬����
          
    
    
    
    
    %% ·�������
    % =========================================================================
    % ���룺��ǰ��ÿ�Ҵ���λ��
    % �����ÿһ�ұ������е�ÿһ��Ŀ�괬��·����WP��
    % �������裺
    % 1.����ÿһ�Ҵ����Ĵ�ͷ��β������waypoint(WP),��ŷֱ�Ϊ0(��ͷ��)��1(��β��);
    % 2.�����⴬Ϊ���ĽǶȣ���8������(000-111)����1000��1111������forѭ������ʮ��������ת����2���Ʊ�ŵķ�ʽ����8������
    % 3.8�������е�ÿһ��������3���㣬���ݹ�ʽ���ҵ���3������м��(����3����͵����Ҵ��ľ������Ԫ���η�����)������м����������·����WP
    % ע�⣺
    % 1.��˭�ĽǶȳ��������⣬�������е�Ŀ�괬�ͱ������е�Ŀ�괬���е�������
    % =========================================================================
    
    for OS=1:1:Boat_Num
        k=1;
        for TS=1:1:Boat_Num
            if TS~=OS
                v_os = Boat(OS).speed(end,:);
                course_os = Boat(OS).COG_deg(end,:);
                pos_os = Boat(OS).pos(end,:);
                v_ts = Boat(TS).speed(end,:);
                course_ts = Boat(TS).COG_deg(end,:);
                pos_ts = Boat(TS).pos(end,:);
                TSlength = ShipSize(TS,1);
                d_thre = 1*1852;  %d_threΪ�ж���ײ���յķ�����ֵ
                changeLabel = 0;
                % =========================================================================
                % ���ж�����ײ���գ�����Ҫ����waypoint��
                % �жϱ�׼���Ƿ�����ײ����
                % ע�⣺���ܳ���ԭ�������б���Ӧ���Ѿ���ȥ������Ԥ��·������ڹ���Ĵ�ͷһ������
                % ��ʱ�Ʋ���������������ݲ���A*����������ݵ�ԭ���������һ�ԣ���Ҫ��������
                % =========================================================================
                CurrentRisk=CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre);
                if CurrentRisk==1  %����ȷ����ײ����
                    WP_label(k) = TS;
                    % ��ʼ����waypiont
                    Dis_temp(k) = norm(pos_os-pos_ts);
                    WayPoint_temp0 = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
                    WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0�Ǵ�ͷ��
                    WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1�Ǵ�β��
                    k=k+1;
                end
            end
        end
        WP_Num = length(WP_label);
        kk = 1;
        for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %��WP_Num�ҷ��մ�����2^WP_Num������������WP_Numλ
            CAL_temp = dec2bin(scenario);
            for ts_i=1:1:WP_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
                if CAL_temp(ts_i)==0
                    WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                elseif CAL_temp(ts_i)==1
                    WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP1;
                end 
            end
            switch WP_Num
                case 1
                    WayPoint_OS(kk,:) = WayPoint_temp1;
                case 2
                    WayPoint_OS(kk,:) = WP_3Ship(WayPoint_temp1(1,:),Dis_temp(1),WayPoint_temp1(2,:),Dis_temp(2));
                case 3
                    WayPoint_OS(kk,:) = WP_4Ship(Dis_temp,WayPoint_temp1);    
            end
            kk=kk+1;
        end
        [m_wp,n_wp] = size(WayPoint_OS);
        t_wp=t*ones(m_wp,1);
        Boat(OS).WP_data = [Boat(OS).WayPoint;t_wp,WayPoint_OS]; %�����͵ó�����tʱ�����г������ۺ�waypoint
        Boat(OS).WayPoint = WayPoint_OS;
    end
    %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
    
    
    
    
    %% ���Ƶ�ǰÿ�Ҵ���SCR
    for i=1:1:Boat_Num
        
        Boat_x = Boat(i).pos(end,1);
        Boat_y = Boat(i).pos(end,2);
        Boat_theta = -Boat(i).COG_rad(end,:); %�˴�Ϊ������
        Boat_Speed = Boat(i).SOG(end,:);
        Shiplength = ShipSize(i,1);
        
        Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
        
    end
    
    %% APF��ͼ���Գ���
    % =========================================================================
    % SCR=zeros(m,n);
    % for i=1:1:Boat_Num
    %     SCR=SCR+Boat(i).SCR;
    % end
    %
    % % figure
    % % kk1=mesh(X,Y,SCR);
    % % colorpan=ColorPanSet(6);
    % % colormap(colorpan);%����ɫ��
    % % axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0 150])
    % % set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % % set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % % set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % % set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % % grid on;
    % % xlabel('\it n miles', 'Fontname', 'Times New Roman');
    % % ylabel('\it n miles', 'Fontname', 'Times New Roman');
    % % title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
    % % box off;
    %
    % figure
    % [M,c] = contourf(X,Y,SCR);
    % c.LineWidth = 0.001;
    % % c.colormap=colorpan;%����ɫ��
    % colorpan=ColorPanSet(6);
    % colormap(colorpan);%����ɫ��
    %
    % for i=1:1:Boat_Num
    %     hold on
    %     ship_icon(Boat(i).pos(1,1),Boat(i).pos(1,2),ShipSize(i,1),ShipSize(i,2),Boat(i).COG_deg,0);
    % end
    % % axis([xmin xmax ymin ymax zmin zmax])
    % axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
    % set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    % set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    % grid on;
    % xlabel('\it n miles', 'Fontname', 'Times New Roman');
    % ylabel('\it n miles', 'Fontname', 'Times New Roman');
    % title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
    % box on;
    % =========================================================================
    
    %% A*�㷨������
    for i=2:1:2
        start_x = round((Boat(i).pos(end,1)+MapSize(1)*1852)/Res);
        start_y = round((Boat(i).pos(end,2)+MapSize(2)*1852)/Res);
        start_theta = Boat(i).COG_rad(end,:);   %��ʼ�����򣬻�����
        %A*�㷨��Ŀ��㣬��ÿ��ʱ�̣����·�����Ӧ��������·���㣬����·����󣬲���������յ�Ŀ���
        end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
        end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);
        
        RiskMap=zeros(m,n);
        for k=1:1:Boat_Num
            if k~=i
                RiskMap=RiskMap+Boat(k).SCR;
            end
        end
        
        ShipLong=2*round(ShipSize(i,1)/Res/2);     %�����ȳ���2ȡ���ٳ�2��Ϊ�˳������õ�ShipLong/2ʱ,Ҳ���Ա�֤Ϊ����
        Movelength=round((Boat(i).speed(end,:)*60)/Res);  %����,ÿ�����н�����
        SurroundPointsNum=20; %������������n���A*
        valueAPF=2;  %APF�Ƴ��ļ�ֵ����
        NodeOpti=0;
        [posData,courseData,courseData_deg] = AstarMain(RiskMap,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
        
        Boat(i).AsPos=posData;
        Boat(i).AsCourse=courseData;
        Boat(i).AsCourse_deg=courseData_deg;
        for ii=1:1:length(Boat(i).AsPos)
            
            Boat(i).DecPos(ii,1)=Boat(i).AsPos(ii,1)*Res-MapSize(1)*1852;
            Boat(i).DecPos(ii,2)=Boat(i).AsPos(ii,2)*Res-MapSize(2)*1852;
            
        end
    end
    
    % %% A*����·��
    % =========================================================================
    % SCR=zeros(m,n);
    % for i=1:1:Boat_Num
    %     SCR=SCR+Boat(i).SCR;
    % end
    %
    % figure
    % kk2=contourf(X,Y,Astar_map);  %�������ɫ�ĵȸ���ͼ
    % colorpan=ColorPanSet(6);
    % colormap(colorpan);%����ɫ��
    % for i=1:1:1
    %     hold on
    %     plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
    %     hold on;
    %     ship_icon(Boat(i).pos(1,1),Boat(i).pos(1,2),ShipSize(i,1), ShipSize(i,2), Boat(i).speed,0);
    %
    %     for ii=1:1:length(Boat(i).AsPos)
    %         hold on;
    %         ship_icon(Boat(i).AsPos(ii,1),Boat(i).AsPos(ii,2),ShipSize(i,1), ShipSize(i,2), Boat(i).AsCourse_deg(ii),1 );
    %     end
    % end
    % =========================================================================
end
toc
disp(['��������ʱ��: ',num2str(toc)]);
