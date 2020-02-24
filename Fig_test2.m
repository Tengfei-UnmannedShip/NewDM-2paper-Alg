%���ڵ�����·���滮����
clear

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
end
%��ͼ����
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=50;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%�������߲�������
for i=1:1:Boat_Num
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).FMPos=[];
    Boat(i).FMCourse=[];
    Boat(i).FMCourse_deg=[];
    Boat(i).Current_row=0;
    Boat(i).RiskHis=[];
    
end

OS=1;
Boat(OS).CAL=CAL0(OS,:);

%% ���������ĺ��г�
% ���Ƶ�ǰ��������Ŀ�괬��SCR

SCR_temp=zeros(m,n);
CAL_Field=zeros(m,n);
ScenarioMap=zeros(m,n);
PeakValue=100;
%0223 �������鷢�֣����м�λ�õ�ʱ�򣬼��Ҵ��ĺ����ܹ��죬4�Ŵ�������ʼ�𵴣�
% ����ԭ��Ӧ�������м�λ�ø����ϰ���ĸ��ֳ��ĵ��ӷǳ����أ������޷���������
% Ŀǰ��������ײ����DCPA���жϣ����û����ײ���գ��򲻻����Ƴ�ͼ��ֻ���Ʊ�����Լ����
RiskLabel=[];
for TS=1:1:Boat_Num
    if TS~=OS
        
        v_os = Boat(OS).speed(end,:);
        course_os = Boat(OS).COG_deg(end,:);
        pos_os = Boat(OS).pos(end,:);
        v_ts = Boat(TS).speed(end,:);
        course_ts = Boat(TS).COG_deg(end,:);
        pos_ts = Boat(TS).pos(end,:);
        d_thre = 1*1852;                % d_threΪ�ж���ײ���յķ�����ֵ
        
        Col_Risk= CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre,1500);
        RiskLabel(TS)=Col_Risk;
        if  Col_Risk==1 %����ײ����Ϊ1
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
            
            %������������µķ��ճ�������RuleField
            CAL=Boat(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField2( pos_ts(1),pos_ts(2),Boat_theta,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
            ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        end
    else
        RiskLabel(TS)=3;
    end
    
end


Boat(OS).RiskHis=[Boat(OS).RiskHis;RiskLabel];
Boat_x=Boat(OS).pos(1,1);
Boat_y=Boat(OS).pos(1,2);
Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
Shiplength = ShipSize(OS,1);
alpha=30;
R=500;
AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
ScenarioMap=1+ScenarioMap+AFMfiled;
FM_map=1./ScenarioMap;

%% FM�㷨������
start_point(1,1) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
start_point(1,2) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);

end_point(1,1) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
end_point(1,2) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);

%����ʼ����յ㵽�߽紦ʱ����Ϊ���ڲ�
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
[Mtotal, paths] = FMM(FM_map, end_point', start_point');
Boat(OS).FM_lable=Boat(OS).FM_lable+1;
path0 = paths{:};
path0 =path0';

posData = zeros(size(path0));
posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
Boat(OS).path=posData;

%% ��ͼ
% ��ͼ����
figure;
kk1=mesh(X,Y,ScenarioMap);
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
axis equal
axis off

figure
kk2=contourf(X,Y,ScenarioMap-1);  %�������ɫ�ĵȸ���ͼ
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
hold on

plot(Boat(1).HisPos(1,1),Boat(1).HisPos(1,2),'ro');
hold on
plot(Boat(1).goal(1),Boat(1).goal(2),'r*');
hold on
plot(Boat(1).path(:, 1), Boat(1).path(:, 2), 'r-');
hold on

plot(Boat(2).HisPos(1,1),Boat(2).HisPos(1,2),'bo');
hold on
plot(Boat(2).goal(1),Boat(2).goal(2),'b*');
hold on

plot(Boat(3).HisPos(1,1),Boat(3).HisPos(1,2),'go');
hold on
plot(Boat(3).goal(1),Boat(3).goal(2),'g*');
hold on

plot(Boat(4).HisPos(1,1),Boat(4).HisPos(1,2),'ko');
hold on
plot(Boat(4).goal(1),Boat(4).goal(2),'k*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;