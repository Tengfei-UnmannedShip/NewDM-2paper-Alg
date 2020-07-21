clear
clc
% close all

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
    0.0, 0.0,  20,    0,    3,  6
    0.0, 0.0,  20,  230,    4,  6
    0.0, 0.0,  18,  300,    5,  6
    0.0, 0.0,  17,  135,    5,  6
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
% ��ʼ���������д���ȫ������
% ����CAL�Ĺ涨��ÿ��ʹ�õĶ���һ�У�����OS��TS��CAL��
% ����CAL0(1,2)=0����˼�Ǳ�����TS��̬����0��������ʲô����������stand-on ship��OSҪ��TS�Ĵ�ͷ��
CAL0=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];
% ���ݳ�ʼ״̬��������õķ��ո�֪��������ֵԽС·�������ô�Խ������ΪOS����ΪTS
% ����ԭ���ǰ��ճ�ʼ״̬��ÿ�Ҵ������������ȱ��ñ����Ҳ�Ĵ���������Ҳബϵ��Ϊ1
Risk_value=[5,1,10
    10,5,1
    10,1,5
    1,10,5];
 syms x y
 
Boat_Num=4;   %��������
tMax=3600;    %���ʱ��
tBayes=2000;  %��Ҷ˹�Ʋ�ʱ��
t_count11=0;    %ʱ�����
t_count12=0;    %ʱ�����
t_count21=0;    %ʱ�����
t_count22=0;    %ʱ�����

%��ͼ����
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %�����־������Ŀ���ʱΪ0��δ����ʱΪ1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    Boat(i).HisCOG=[];
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    pos0=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    %�ѳ�ʼλ�ù�һ��դ��λ���ϣ�������Ȼ������
    Boat(i).pos(1,1) = round(pos0(1,1)/Res)*Res;
    Boat(i).pos(1,2) = round(pos0(1,2)/Res)*Res;
    Boat(i).RiskHis = [];
    Boat(i).HisPos=[];
    %��һ�ξ��ߵ�������
    Boat(i).path = [];
    Boat(i).infercount=0;
end


%�������߲�������
for i=1:1:Boat_Num
    %��Ŀ���λ�ù�һ��դ��λ���ϣ�������Ȼ������
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    Boat(i).waypoint=[];
    Boat(i).WP1=[];
    Boat(i).WP2=[];
    Boat(i).End=[];
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).decision_delay=0; %��ʼʱ��decision_delayΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).Current_row=0;
    for ii=1:1:Boat_Num
        %infer_label�Ƕ�Ŀ�괬���Ʋ�������ۼƣ���ʽΪ[�ϴ��Ʋ��ʱ�̣�TS�ܵ��Ʋ����]
        Boat(i).InferData(ii).infer_label=[0,0];
    end
    Boat(i).Theta_his=[];
    %ÿ�Ҵ���CAL����
    Boat(i).CAL=CAL0(i,:);       % Boat(OS).CAL��һ����������ʾOS������TS��CAL
    Boat(i).CAL_infer=CAL0(i,:); % Boat(OS).CAL_infer��һ�����������ڴ洢�Ʋ��CAL
end

%% Ԥ���㣬���ݳ�ʼ��������ÿ�Ҵ�������waypoint����дwaypoint�����˲�ͬ�ı��������������Ժ󱣳ֲ��䣬ֱ��CAL�ı�
for  OS=1:1:Boat_Num
    v_os          = Boat(OS).speed;
    course_os = Boat(OS).COG_deg;
    pos_os      = Boat(OS).pos;
    for TS=1:1:Boat_Num
        if TS~=OS
            v_ts          = Boat(TS).speed;
            course_ts = Boat(TS).COG_deg;
            pos_ts      = Boat(TS).pos;
            % ��һ����ÿ�Ҵ�����waypoint����
            WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,ShipSize(TS,1),0);
            %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
            Boat(OS).WP1 = [Boat(OS).WP1; WayPoint_temp(1,1:2)];
            %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
            Boat(OS).WP2 = [Boat(OS).WP2; WayPoint_temp(1,3:4)];
        end
    end
    WP_Num=Boat_Num-1;
    WayPoint_OS=[];
    for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %��WP_Num�ҷ��մ�����2^WP_Num������������WP_Numλ
        CAL_temp = dec2bin(scenario);
        for ts_i=1:1:WP_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
            CAL_ts=str2num(CAL_temp(ts_i+1));
            %�˴���ͼ4.1-1Ϊ��׼���������У�����000����˼�ǣ����д����ǶԱ�����·�������ӶԷ���ͷ����CAL���Ǵ�����
            if CAL_ts==0
                WayPoint_temp1(ts_i,:) = Boat(OS).WP1(ts_i,:);  %��ͷ·����
            elseif CAL_ts==1
                WayPoint_temp1(ts_i,:) = Boat(OS).WP2(ts_i,:);  %��β·����
            end
        end
        %�ҵ���ǰ�����е�waypoint������8���е�һ��
        %         WayPoint_OS0=WP_4Ship(Risk_value(OS,:)',WayPoint_temp1);

        WayPoint_OS0 = ScenarioWaypoint( Risk_value(OS,:)',WayPoint_temp1);
        Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS0];
    end   %��ɶԵ�ǰOS����8��·������ռ�
    
end

%ʶ���ʼ����
for  OS=1:4
    k=2;
    scen_os=1;
    for TS=1:4
        if TS~=OS
            scen_os=scen_os+CAL0(OS,TS)*(2^k);
            k=k-1;
        end
    end
    Boat(OS).currentWP=Boat(OS).waypoint(scen_os,:);
end

toc
figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end
for  scen=1:8
    plot(Boat(1).waypoint(scen,1),Boat(1).waypoint(scen,2),'r*');
end

for  scen=1:8
    plot(Boat(2).waypoint(scen,1),Boat(2).waypoint(scen,2),'b^');
end

for  scen=1:8
    plot(Boat(3).waypoint(scen,1),Boat(3).waypoint(scen,2),'gp');
end

for  scen=1:8
    plot(Boat(4).waypoint(scen,1),Boat(4).waypoint(scen,2),'kh');
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

%������ǰ����

figure
hold on
for draw=1:4
    ship_icon( Boat(draw).pos(1,1),Boat(draw).pos(1,2),3*ShipSize(draw,1),3*ShipSize(draw,2),Boat(draw).COG_deg,draw )
end

plot(Boat(1).currentWP(1),Boat(1).currentWP(2),'r*');
plot(Boat(2).currentWP(1),Boat(2).currentWP(2),'b*');
plot(Boat(3).currentWP(1),Boat(3).currentWP(2),'g*');
plot(Boat(4).currentWP(1),Boat(4).currentWP(2),'k*');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;