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
% �ɵ��߼�01�Ƿ���
% CAL0=[
%     2 0 0 1
%     1 2 0 1
%     1 1 2 0
%     0 0 1 2];

CAL0=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];


Boat_Num=4;   %��������
tMax=3600;    %���ʱ��
tBayes=2000;  %��Ҷ˹�Ʋ�ʱ��
t_count11=0;    %ʱ�����
t_count12=0;    %ʱ�����
t_count21=0;    %ʱ�����
t_count22=0;    %ʱ�����
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
Res=18.52*2;  %Resolution��ͼ�ķֱ���

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
Boat(OS).CAL=CAL0(OS,:);

%% ���������ĺ��г�
% ���Ƶ�ǰ��������Ŀ�괬��SCR
t_count31=toc;    %ʱ�����
SCR_temp=zeros(m,n);
CAL_Field=zeros(m,n);
ScenarioMap=zeros(m,n);
AFMfield=zeros(m,n);
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
        d_thre = 1*1852;                % d_threΪ�ж���ײ���յķ�����ֵ
        
        CPA_temp=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA_temp=CPA_temp(5);
        %���DCPA_temp=0�����������0��Ϊ���������£������С��DCPA����Ϊ1�����̫С�Ļ��������ܴ�Ӱ��ǰ��Ĳ���
        if DCPA_temp<1
            DCPA_temp=1;
        end
        TCPA_temp=CPA_temp(6);
        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
            CurrentRisk=1; %����ײ����Ϊ1
        else
            CurrentRisk=0;
        end
        Dis_temp=norm(pos_os-pos_ts);
        risklevel=0;
        %���㵱ǰ��Ŀ�괬�����޷��ա�TCPA��DCPA��������ľ���
        Risk_temp(k,:) = [TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp,risklevel];
        
        k=k+1;
        if  CurrentRisk==1 %����ײ����Ϊ1
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
            
            %������������µķ��ճ�������RuleField
            
            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
            disp(['������',num2str(Boat(OS).COG_deg),'����',num2str(OS),'�Ŵ���',num2str(Boat(TS).COG_deg),'���н�Ϊ',num2str(cro_angle)]);
            
            CAL=Boat(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,50,CAL);
            ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        end
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
kk2=contourf(X,Y,ScenarioMap);  %�������ɫ�ĵȸ���ͼ
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
% set(kk2, 'LineStyle','none');
hold on
plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
hold on
%                     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );


t_count32=toc;
disp([num2str(OS),'�Ŵ����㺽�г���ʱ: ',num2str(t_count32-t_count31)]);
%%Ѱ��ָ����

TCPA_OS=sum(Risk_temp(:,3));
DCPA_OS=sum(Risk_temp(:,4));
Dis_OS=sum(Risk_temp(:,5));
Risk_OS=Risk_temp;

Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
%�����Ŀ�����ҵ���������ֵ�����õķ������ۺϷ���TCPA����ȣ�Ȼ��Dis��Ȼ����DCPA��
%��100��10��1��Ϊϵ�����ֿ������ǲ�֪��Ч����Σ���Ҫ��һ���ĵ���
Risk_OS(:,6)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
t_risk=t*ones(size(Risk_OS,1));


%% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
Risk_level=0;
Danger_TS=OS;
Risk_count=0;
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        if Risk_OS(k,2)~=0
            Risk_count=Risk_count+1;
            if Risk_OS(k,6)>Risk_level
                Risk_level=Risk_OS(k,6);
                Danger_TS=Risk_OS(k,1);
            end
        end
        k=k+1;
    end
    
end

%% FM�㷨������
start_point(1,2) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
start_point(1,1) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);


if  Danger_TS==OS   %����ǰû�з��մ���Ŀ���Ϊ�յ�
    
    end_point(1,1) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
    end_point(1,2) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
    disp('��ǰû�з��մ���Ŀ���Ϊ�յ�');
    
else     %��Σ�յĴ���Ϊ��ǰ��Ŀ��λ��·����
    disp(['��ǰ���մ���',num2str(Risk_count),'�ң���Σ�յĴ�Ϊ',num2str(Danger_TS)]);
    if   size(Boat(OS).End,1)>=1 && Boat(OS).End(end,2)==Danger_TS   %���ǵ�һ�ξ�����Σ�մ����ϴ���ͬ
        end_point= Boat(OS).End(end,3:4);
    else
        v_ts = Boat(Danger_TS).speed(end,:);
        course_ts = Boat(Danger_TS).COG_deg(end,:);
        pos_ts = Boat(Danger_TS).pos(end,:);
        TSlength = ShipSize(Danger_TS,1);
        changeLabel = 0; %���ɱ�·����
        
        WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel);
        %         WayPoint_temp = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
        if Boat(OS).CAL(Danger_TS)==0 %��ʱ�����Ը�Ŀ�괬��0��������ΪStand-onֱ����
            %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
            WP= WayPoint_temp(1,1:2);
            disp('·����Ϊ��ͷ��');
        elseif Boat(OS).CAL(Danger_TS)==1  %��ʱ�����Ը�Ŀ�괬��1��������ΪGive-way��·��
            %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
            WP= WayPoint_temp(1,3:4);
            disp('·����Ϊ��β��');
        end
        
        end_point(1,2) =round((WP(1,1)+MapSize(1)*1852)/Res);
        end_point(1,1) =round((WP(1,2)+MapSize(2)*1852)/Res);
        
    end
end

% end_point=[695,271];
Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];

% end_point(1,2)=round((-7.5*1852+MapSize(1)*1852)/Res);
% end_point(1,1)=round((0*1852+MapSize(2)*1852)/Res);


if size(Boat(OS).End,1)>1 && norm(Boat(OS).End(end-1,3:4)-end_point)<=2 ... %���ǵ�һ������Ļ�����Ҫ�ж��Ƿ�Ҫά���ϴεľ���
        && Boat(OS).Current_row <= size(Boat(OS).path,1)-10         %��Ԥ���ߵ�pathֻʣ����30����ʱ��ǿ�����¼���
    disp('Ŀ��λ�ñ䶯�����򲻼���');
else
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

    [Mtotal, paths] = FMM(FM_map, end_point',start_point');
    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
    FinalMap=Mtotal;
    FMpath0 = paths{:};
    FMpath =FMpath0';
    path0=fliplr(FMpath);
    
    posData = zeros(size(path0));
    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
    end_point1(1)=end_point(2)*Res-MapSize(1)*1852;
    end_point1(2)=end_point(1)*Res-MapSize(2)*1852;
    Boat(OS).path=posData;
    Boat(OS).Current_row=1;   %ÿ�����¾��ߣ���ǰ������1
    Boat(OS).decision_count=1;
    
end


% ��ͼ
% ��ͼ����
figure
mesh(X,Y,FM_map);
title('��ǰ�����ͼ')
figure
mesh(X,Y,RiskMap);
title('��ǰ������ȫͼ')
figure
mesh(X,Y,GuidanceMap);
title('��ǰ����ͼ')
figure
mesh(X,Y,FinalMap)

figure
kk0=contourf(X,Y,FinalMap);  %�������ɫ�ĵȸ���ͼ
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
hold on
plot(end_point1(1),end_point1(2),'r*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title('��ǰFM����ͼ')

box on;




figure
kk2=contourf(X,Y,safetymap);  %�������ɫ�ĵȸ���ͼ
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
hold on
plot(end_point(1),end_point(2),'r*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;