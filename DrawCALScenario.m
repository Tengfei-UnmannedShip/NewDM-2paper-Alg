%% ר�����ڻ���ship1���еĸ����������
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

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40
    ];

CAL0=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];

Boat_Num=4;   %��������
Start_pos=[];
t=1;
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
Res=18.52;  %Resolution��ͼ�ķֱ���

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
    Boat(i).End=[];
    
end
tic
OS=1;

for scenario=0:1:0
    CAL_temp=dec2bin(scenario+8);
    CAL_temp2=CAL_temp(2:end);
    CAL_now=2;
    disp(['��ǰ����Ϊ',CAL_temp2]);
    for b=1:3
    CAL_now(b+1)=str2num(CAL_temp(b+1));
    end
    Boat(OS).CAL=CAL_now;
%% ���������ĺ��г�
% ���Ƶ�ǰ��������Ŀ�괬��SCR
SCR_temp=zeros(m,n);
CAL_Field=zeros(m,n);
ScenarioMap=zeros(m,n);
AFMfield=zeros(m,n);
SCR=zeros(m,n);
PeakValue=100;
%0223 �������鷢�֣����м�λ�õ�ʱ�򣬼��Ҵ��ĺ����ܹ��죬4�Ŵ�������ʼ�𵴣�
% ����ԭ��Ӧ�������м�λ�ø����ϰ���ĸ��ֳ��ĵ��ӷǳ����أ������޷���������
% Ŀǰ��������ײ����DCPA���жϣ����û����ײ���գ��򲻻����Ƴ�ͼ��ֻ���Ʊ�����Լ����
RiskLabel=[];
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        
        v_os = Boat(OS).speed(end,:);
        course_os = Boat(OS).COG_deg(end,:);
        pos_os = Boat(OS).pos(end,:);
        v_ts = Boat(TS).speed(end,:);
        course_ts = Boat(TS).COG_deg(end,:);
        pos_ts = Boat(TS).pos(end,:);
        
        k=k+1;
        
        Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
        Boat_Speed = Boat(TS).SOG(end,:);
        Shiplength = ShipSize(TS,1);
        
        SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
        
        %������������µķ��ճ�������RuleField
        cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
        CAL=Boat(OS).CAL(TS);
        Rule_eta=2;
        Rule_alfa=0.1;
        %             CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
        CAL_Field= RuleField3(pos_ts(1),pos_ts(2),Boat_theta,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        SCR=SCR+SCR_temp;
    else
        RiskLabel(TS)=3;
    end
end
RiskLabel=[1,RiskLabel];
Boat(OS).RiskHis=[Boat(OS).RiskHis;RiskLabel];
RiskMap=1./(ScenarioMap+1);

% ���Ƶ�ǰ�����ĺ�������
Boat_x=Boat(OS).pos(1,1);
Boat_y=Boat(OS).pos(1,2);
Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
Shiplength = ShipSize(OS,1);
alpha=30;
R=500;
AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
GuidanceMap=1./(AFMfield+1);
safetymap=AFMfield+ScenarioMap;
SMap=1./(safetymap+1);
FM_map=10*min(RiskMap, GuidanceMap); % a, b�Ǿ���

%% ��ͼ����
figure(scenario+1);
name=[CAL_temp2,'.jpg'];

kk1=mesh(X,Y,SCR);
% kk1=mesh(X,Y,ScenarioMap);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
draw=OS;
ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,0 )
axis equal

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0  300])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'FontSize',20);
xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',20);
ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',20);
zlabel('Risk Value', 'Fontname', 'Times New Roman','FontSize',20);
box on;
grid on;
% view([0,90]);
% ����ͼ��Ϊjpg
% h = figure(scenario+1);
% 
% saveas(h,name);




% figure
% kk2=contourf(X,Y,ScenarioMap);  %�������ɫ�ĵȸ���ͼ
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% hold on
% plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
% draw=OS;
% ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
% title('��ǰ����Ϊ')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
end
% figure
% mesh(X,Y,GuidanceMap);
% title('����ͼ')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
%
% figure
% mesh(X,Y,AFMfield);
% title('����ͼ')
% axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
% set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
% set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
% grid on;
% xlabel('\it n miles', 'Fontname', 'Times New Roman');
% ylabel('\it n miles', 'Fontname', 'Times New Roman');
% box on;
