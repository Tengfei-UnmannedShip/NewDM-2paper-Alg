%% A*��APF��ϵĵڶ��棬A*������������ʼ����ݱ�����ship4�����ñ仯
% A*�㷨ԭ������Ѧ˫��
% ����ĵ�һ������д��ڵ����⣺
% 0.û���м��??�м��Ӧ������APF��ͼ����֮��ģ����ԣ�APFӦ��Ҳ��һ������
% 1.��ͼ�Ƿ���??����ת��
% 2.��ɫΪ�ڰ�-�Ƿ��·��ȡ�����ŵ��ȸ���ͼ����ʾ���ο�classicAPF/new-main��ķ���

clear all
clc;
close all
tic;%tic1
MapSize=[300,300];
GoalRange=MapSize-[15,15];
[X,Y]=meshgrid(-MapSize(1):1:MapSize(1),-MapSize(2):1:MapSize(2));
[m,n]=size(X);
%%==================================================
%����������������
% ����ģ�Ͳο��Ŷռ��㷽��
% ==================================================
Boat_Speed_Factor=1;        %�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��
BoatRiskFieldPeakValue=100;   %�������ֵ���Ը�����Ҫ��������
Boat_eta=1;
Boat_alfa=0.1;
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue����ÿһ����Ƴ�ֵ���˴��Ƚ������Ϊһ����X��ͬ��С��0����

U_att=zeros(m,n);
APFValue=zeros(m,n);

Boat_Num=3;%��������
%�۲���λ��
Boat.State(1,:)=[   0   220   180     5  30  10];  %����״̬���꣨x,y,ang,v,l,w��angΪ�������� vΪ�ٶȴ�С,lΪ������wΪ����
Boat.State(2,:)=[-200    50   120     5  20   6];  %����״̬���꣨x,y,ang,v��angΪ��������
Boat.State(3,:)=[ 150    70  -120     8  10   3];
Boat.State(4,:)=[ 120  -150   -45    10  20   5];  %ship4�Ǳ���
% Boat.State(4,:)=[ -70   -15    90    10  20   5];

goal=Goal_point(Boat.State(4,1),Boat.State(4,2),Boat.State(4,3),GoalRange);
k_near=10;%����Ŀ��㸽��������Ҫ������ϵ��
k_far=10; %���������ż�֮�������㶨ʱ������ϵ����һ����k_near��ͬ���������ż��������������ͻ��
Po_att=100;%�����ż��뾶
k=0.3;%����ϵ��
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);

for i=1:1:Boat_Num

    %���������ʼ��
    Boat_x(i)=Boat.State(i,1);                  %��i����x����
    Boat_y(i)=Boat.State(i,2);                  %��i����y����
    Boat_theta(i)=-Boat.State(i,3)/180*pi;       %��i��������Ƕ�
    Boat_Speed(i)=Boat.State(i,4);              %��i�����ٶȴ�С
    Boat_length(i)=Boat.State(i,5);             %��i������
    Boat_width(i)=Boat.State(i,6);              %��i������
    
    %�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX{i},BoatY{i}��
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    BoatSpeedFactor{i}=Boat_Speed_Factor*Boat_Speed(i);
    
    %����ռ��е㵽��i�������ؾ���Dis{i}
    Dis{i}=zeros(m,n);
    Dis{i}=sqrt(BoatX{i}.^2+BoatY{i}.^2).*(BoatY{i}<=0)+sqrt((BoatY{i}/(Boat_Speed(i)+1)).^2+BoatX{i}.^2).*(BoatY{i}>0);
    
    %�����i�������ճ�
    BoatRiskField{i}=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis{i})./(Boat_alfa*Dis{i}));
    
    if  BoatRiskField{i}>BoatRiskFieldPeakValue
        BoatRiskField{i}=BoatRiskFieldPeakValue;
    end
    BoatRiskField{i}(BoatRiskField{i}>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;
    
    if BoatCut==1
        BoatRiskField{i}=BoatRiskField{i}.*(BoatX{i}>=0)+0.*(BoatX{i}<0);
    end
    
    %����ÿ�����ڲ�ͬ���µĳ�֮���ݲ��ü򵥼Ӻ�
    RiskFieldValue=RiskFieldValue+BoatRiskField{i};
end
% APFValue=max(U_att,RiskFieldValue);
APFValue=RiskFieldValue;
newfield=RiskFieldValue/BoatRiskFieldPeakValue;
figure;
kk1=mesh(X,Y,APFValue);
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),1 );
axis equal
axis off
%     surf(X,Y,APFValue);

figure
% kk2=pcolor(APFValue);
kk2=contourf(X,Y,APFValue);  %�������ɫ�ĵȸ���ͼ
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
% set(kk2, 'LineStyle','none');
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),1 );
% axis equal
% axis off



%% A*�㷨��ʼ
% function [SetClose,SetOpen]=CircleAStar(map,start_y,start_x,end_y,end_x)
start_x = Boat.State(4,1)+300;
start_y = Boat.State(4,2)+300;
end_x = goal(1,1)+300;
end_y = goal(1,2)+300;

map=newfield;

%% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_y,start_x),Ŀ���ͼ������(destination_y, destination_x),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
%% line1. ���ó�ʼ��������
background=map;
start_point.y=start_y;
start_point.x=start_x;
destination.y=end_y;
destination.x=end_x;
ShipLong=4;
Movelength=20;  %����
SurroundPointsNum=20; %������������n���A*
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %ת��뾶
valueAPF=2;  %APF�Ƴ��ļ�ֵ����

%��ʼ����
%% line2. ��ʼ׼����
%�������λ���ڵ�ͼ��Χ֮����ߴ���״̬����ȫ���㷨��������ʾ�ް�ȫ·����
%���������ʼ�ڵ������(���ꡢ���򡢳ͷ�ֵ���ƶ�����G����Ŀ����Ԥ�ƴ���H���ܴ���F����һ���ƶ�����r�����ڵ㡢�ӽڵ��)
%�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
if (0<start_point.x<length(background(1,:))&&0<start_point.y<length(background(:,1)))
    start_point.G=0; %�ƶ����� G
    start_point.H=sqrt((destination.x-start_point.x)^2+(destination.y-start_point.y)^2);  %��Ŀ����Ԥ�ƴ���H
    start_point.F=start_point.G+start_point.H; %�ܴ���F
    start_point.R= Movelength; %��һ���ƶ�����r
    start_point.Dir=pi/2;  %��ʼ������
    SetOpen(1)=start_point; %��ʼ������
    SetOpen(1).father=nan; %���ڵ�
    SetClose(1)=SetOpen(1); %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
end
%% ��ʼ����
while  ~isempty(SetOpen)  %line3.While: open �б�Ϊ��
    for ii=2:length(SetOpen)  %line4.Ѱ��open�б���Fֵ��С�Ľڵ㣬��ΪFMin��
        if SetOpen(ii).F < SetOpen(1).F
            a=SetOpen(ii);
            SetOpen(ii)=SetOpen(1);
            SetOpen(1)=a;
        end
    end
    SetClose=[SetClose;SetOpen(1)]; %line5-1.��FMin����close�б�,����FMin����SetClose(end),ͬʱ��open�б���ɾ���õ㣻
    SetOpen(1)=[]; %line5-2.��FMin����close�б�ͬʱ��open�б���ɾ���õ㣻
    Surround=[];
    
    %         %% �㷨4���ڽ��ڵ���ѡ
    %         %���룺A*�㷨�е�close�б�
    %         %������Ż����close�б�
    %         %%%��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         L_Close=length(SetClose);
    %         ComPoint=[];
    %         if L_Close>2
    %             ComPoint=SetClose(end).father;
    %             while ~(ComPoint.y==start_y && ComPoint.x==start_x)
    %                 if ((SetClose(end).y-ComPoint.y)^2+(SetClose(end).x-ComPoint.x)^2)<(ComPoint.R)^2
    %                     SetClose(end).father=ComPoint;
    %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.y,ComPoint.x);
    %                 end
    %                 ComPoint=ComPoint.father;
    %             end
    %         end
    %         SetClose(end).father=ComPoint;
    
    %% %%�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. ���� FMin �ڵ�ĳͷ�ֵ��С������һ��Ӧ���ƶ��Ĳ��� ShipSpeed��
    ShipSpeed=Movelength * (1-map(SetClose(end).y,SetClose(end).x));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.���㴬���ƶ�һ���ľ������movecost��Ӧ��չ������ڵ���Num��
    movecost=10; %���Ϊ���ٵ�A*������movecost�������ı�
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For ���ɵ�ÿһ������ڵ�Surround(i)��
        Surround(ii).y=floor(SetClose(end).y-ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).x=floor(SetClose(end).x+ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).y,SetClose(end).x,Surround(ii).y,Surround(ii).x);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%���ټ������ڵ������
        if ~isempty( SetOpen)
            openitear=1;
            mindis = 1000;
            while (openitear<length(SetOpen))
                dis=sqrt((Surround(ii).y -SetOpen(openitear).y)^2+(Surround(ii).x-SetOpen(openitear).x)^2);
                if(dis<mindis)
                    mindis=dis;
                    replace=openitear;
                end
                openitear=openitear+1;
            end
            if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).y,Surround(ii).x,SetOpen(replace).y,SetOpen(replace).x,ShipLong/2)==1)
                %                         if (mindis<6)
                Surround(ii).y=SetOpen(replace).y;
                Surround(ii).x=SetOpen(replace).x;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % line10.If(Sourround(i)��Ŀ������Χ֮�ڣ���Sourround(i)�������ϰ����Sourround(i)����close�б���,�Ҵ�FMin�ƶ���Sourround(i)�����д�����ȫ,���˶����̲��ܴ����˶���������)
        if (0>=Surround(ii).x||Surround(ii).x>=length(background(1,:))||0>=Surround(ii).y||Surround(ii).y>=length(background(:,1))...
                || background(Surround(ii).y,Surround(ii).x)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).y,SetClose(end).x,Surround(ii).y,Surround(ii).x,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).y,Surround(ii).x,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. ����Sourround(i)��G��H��Fֵ,����FMinΪSourround(i)�ĸ��ڵ㣻
            Surround(ii).H=sqrt((destination.x-Surround(ii).x)^2+(destination.y-Surround(ii).y)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).y,Surround(ii).x);%movecost���ڵ����Ƴ�����ֵ
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %����FMinΪSourround(i)�ĸ��ڵ㣻
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)�������겻ͬ��open�б������������)
                SetOpen=[SetOpen;Surround(ii)]; %line13. ��Sourround(i)����open�б�
            else %line14
                %% line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
                for kk=1:length(SetOpen)
                    %                         if abs(Surround(ii).y - SetOpen(kk).y)<=1/4*ShipLong && abs(Surround(ii).x-SetOpen(kk).x)<=1/4*ShipLong
                    if (Surround(ii).y == SetOpen(kk).y && Surround(ii).x==SetOpen(kk).x)
                        rember=kk;                       %�ҵ�Sourround(i)��open�б��о�����ͬ����Ľڵ�
                    end
                end
                if Surround(ii).G < SetOpen(rember).G     %�Ƚ�Gֵ
                    SetOpen(rember).father=SetClose(end); %���ý�С�ߵĸ��ڵ�ΪFMin��
                end
            end %line16.
        end     %line17.
    end         %line18.
    if SetClose(end).H < ShipSpeed %line19. ���FMin��Ŀ���ľ���С���ƶ��������㷨������
        break;
    end
end
destination.father=SetClose(end);
destination.Dir=ShipDirection(SetClose(end).x,SetClose(end).y,end_x,end_y);
%����·��
figure
ss=pcolor(background);  %����pcolor�Ĺٷ�ʾ��
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
set(ss, 'LineStyle','none');
% rectangle('position',[1 1 size(background)-1],'edgecolor','k')%����ͼƬ�߿��С����ɫ
t=1;
M(t)=getframe;
t=t+1;

CurrentPoint=destination;
PosTemp=[];
courseTemp=[];
posData0=[];
courseData=[];
while ~(CurrentPoint.x==start_point.x && CurrentPoint.y==start_point.y)
    position=[CurrentPoint.x  CurrentPoint.y];
    plotShip(fliplr(position),CurrentPoint.Dir,ShipLong/2);
%     ship_icon(position(1),position(2),Boat.State(4,5)/5, Boat.State(4,6)/5, CurrentPoint.Dir,1 );
    PosTemp=[PosTemp;position];
    courseTemp=[courseTemp;CurrentPoint.Dir];
    CurrentPoint=CurrentPoint.father;
    M(t)=getframe;
    t=t+1;
    
end
line([start_point.x-3;start_point.x+3;start_point.x+3;start_point.x-3;start_point.x-3],[start_point.y-3;start_point.y-3;start_point.y+3;start_point.y+3;start_point.y-3],'color','g','LineWidth',5);
line([destination.x-3;destination.x+3;destination.x+3;destination.x-3;destination.x-3],[destination.y-3;destination.y-3;destination.y+3;destination.y+3;destination.y-3],'color','b','LineWidth',5);

posData0=[posData0;flipud(PosTemp)];
deltaPos=300*ones(size(posData0));
posData=posData0-deltaPos;
courseData=[courseData;flipud(courseTemp)];
courseData_deg=180+courseData/pi*180;

figure
kk2=contourf(X,Y,APFValue);  %�������ɫ�ĵȸ���ͼ
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
% set(kk2, 'LineStyle','none');
hold on
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),2 );

for i=1:1:length(posData)
    hold on;
    ship_icon(posData(i,1),posData(i,2),Boat.State(4,5)/5, Boat.State(4,6)/5, courseData_deg(i),1 );
end
% axis equal
% axis off
toc
disp(['��������ʱ��: ',num2str(toc)]);
