clear
clc
close all

tic;%tic1
%% ��ʼ����
% =========================================================================
% ������������
% =========================================================================
% 1.compliance:�������������:0.������COLREGs,1.����COLREGs;
% 2.inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
shipLabel=[
    1 0
    1 0
    1 0
    1 0];
%1~2λ��(�м��λ�ã�������ʼλ��)��3����(��)��4��ʼ����deg������Ϊ0����5��������ʱ����6��ⷶΧ��range��nm��
%����������������
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];
% % ׷Խ��������
% ShipInfo=[
%     0.0,  0.0,  18,    0,  3,  6
%     0.0,  0.0,  13,    0,  4,  6
%     0.0,  0.0,  16,  300,  5,  6
%     0.0,  0.0,  13,  135,  5,  6
%     ];

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40
    ];
%��ʼ���������д���ȫ������
CAL0=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

Boat_Num=4;   %��������
tMax=3600;    %���ʱ��
tBayes=2000;  %��Ҷ˹�Ʋ�ʱ��
t_count11=0;    %ʱ�����
t_count12=0;    %ʱ�����
t_count21=0;    %ʱ�����
t_count22=0;    %ʱ�����
Start_pos=[];
%��ͼ����
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %�����־������Ŀ���ʱΪ0��δ����ʱΪ1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    Boat(i).HisCOG=[Boat(i).COG_rad,Boat(i).COG_deg];
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    Start_pos = [Start_pos;Boat(i).pos];
    Boat(i).HisPos=Boat(i).pos;
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852);
end

% CAL0(1,:)=[2 0 1 1];
t_count11=t_count12;    %ʱ�����
reach_label=0; %ÿ��ʱ�̹���
%% ÿ��ʱ�̵�״̬����
for OS=1:1:1    %Boat_Num
    %�жϵ�ǰiʱ������OS���ľ���������,compliance==1����������,��δ����Ŀ���
    
    %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
    if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
        % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
        Boat(OS).CAL=CAL0(OS,:);
    elseif  shipLabel(OS,2)==1
        % ��Ҷ˹�Ʋ�
        Boat(OS).CAL=CAL0(OS,:); %��Ҷ˹�ƶ����ó��ģ����ǵ�ǰʱ�̵�Boat(i).CAL
    end
    %% ����·���滮������
    % �ڵ�ǰʱ�����¼���FM������������
    %   1.֮ǰ��FM�������Ѿ��þ�����Ҫ���¼��㣻
    
    %% ���������ĺ��г�
    % ���Ƶ�ǰ��������Ŀ�괬��SCR
    t_count31=toc;    %ʱ�����

    PeakValue=100;
    k=1;
    for TS=1:1:Boat_Num
        if TS~=OS
            SCR_temp{k}=zeros(m,n);
            CAL_Field{k}=zeros(m,n);
            ScenarioMap{k}=zeros(m,n);
            Boat_x = Boat(TS).pos(end,1);
            Boat_y = Boat(TS).pos(end,2);
            Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp{k} = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
            
            %������������µķ��ճ�������RuleField
            CAL=Boat(OS).CAL(TS);
            CAL_Field{k} = RuleField( Boat_x,Boat_y,Boat_theta,Shiplength,MapSize,Res,PeakValue,CAL);
            ScenarioMap{k}=SCR_temp{k}+CAL_Field{k};
            k=k+1;

        end
    end
    Scenario=zeros(m,n);
    for i=1:1:k-1
        SCR_temp_max(i)=max((SCR_temp{i}(:)));
        CAL_Field_max(i)=max((CAL_Field{i}(:)));
        ScenarioMap_max(i)=max((ScenarioMap{i}(:)));
        
        Scenario=Scenario+ScenarioMap{i};
        
    end
    
    % ���Ƶ�ǰ�����ĺ�������
    AFMfiled=zeros(m,n);
    Boat_x=Boat(OS).pos(1,1);
    Boat_y=Boat(OS).pos(1,2);
    Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
    Shiplength = ShipSize(OS,1);
    alpha=30;
    AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,Shiplength,MapSize,Res,200);
    Scenario=AFMfiled+Scenario;
    
    %% ��ͼ����
%     figure;
%     kk1=mesh(X,Y,Scenario);
%     colorpan=ColorPanSet(6);
%     colormap(colorpan);%����ɫ��
%     hold on
%     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),2 );
%     axis equal
%     axis off
%     %     surf(X,Y,APFValue);
    
    figure
    % kk2=pcolor(APFValue);
    kk2=contourf(X,Y,Scenario);  %�������ɫ�ĵȸ���ͼ
    colorpan=ColorPanSet(6);
    colormap(colorpan);%����ɫ��
    % set(kk2, 'LineStyle','none');
    hold on
    plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
    hold on
%     ship_icon(Boat(OS).pos(1,1),Boat(OS).pos(1,2),ShipInfo(OS,5)*250,ShipInfo(OS,6)*50, Boat(OS).COG_deg(1),0);
    axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
    set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
    set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
    grid on;
    xlabel('\it n miles', 'Fontname', 'Times New Roman');
    ylabel('\it n miles', 'Fontname', 'Times New Roman');
    box on;
    
end
