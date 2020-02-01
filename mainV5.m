%% (2.5�汾)��WangNing��ShipDomain����ԭAPF����չ������Ϊ��λ�ĵ�ͼ
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.A*��APF��ϵĵ����棬A*��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
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

% ==================================================
% ������������
% ==================================================
%1~2λ��(�м��λ�ã�������ʼλ��)��3����(��)��4��ʼ����deg������Ϊ0����5��������ʱ����6��ⷶΧ��range��nm��
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];

ShipSize = [ 250, 30
    290, 45
    290, 45
    270, 40 ];

Boat_Num=4;%��������
tMax=2500;   %���ʱ��
tBayes=2000;    %��Ҷ˹�Ʋ�ʱ��
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

    
end
% [X,Y]=meshgridmeshgrid(-MapSize(1):Res:MapSize(1),-MapSize(2):Res:MapSize(2));
%% ���Ƶ�ǰÿ�Ҵ���APF
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).pos(1,1);
    Boat_y = Boat(i).pos(1,2);
    Boat_theta = -Boat(i).COG_rad; %�˴�Ϊ������
    Boat_Speed = Boat(i).SOG;
    Shiplength = ShipSize(i,1);

    Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);

end

%% APF��ͼ���Գ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% A*�㷨��ʼ
% valueAPF0=[0.1,0.5,1,2,5,10,50,100];

for i=2:1:2
    start_x = round((Boat(i).pos(1,1)+MapSize(1)*1852)/Res);
    start_y = round((Boat(i).pos(1,2)+MapSize(2)*1852)/Res);
    start_theta = Boat(i).COG_rad;   %��ʼ�����򣬻�����
    %A*�㷨��Ŀ��㣬��ÿ��ʱ�̣����·�����Ӧ��������·���㣬����·����󣬲���������յ�Ŀ���
    end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
    end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);

    RiskMap=zeros(m,n);
    for k=1:1:Boat_Num
        if k~=i
            RiskMap=RiskMap+Boat(k).SCR;
        end
    end
    ShipLong=2*round(ShipSize(i,1)/Res/2);
    Movelength=round((Boat(i).speed*60)/Res);  %����,ÿ�����н�����
    SurroundPointsNum=20; %������������n���A*
    valueAPF=2;  %APF�Ƴ��ļ�ֵ����
    NodeOpti=0;
%     [posData,courseData,courseData_deg] = AstarMain(RiskMap,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);

map=RiskMap;
    %% ��ʽ���㿪ʼ
%     function [posData,courseData,courseData_deg] = AstarMain(map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize)
%% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_y,start_x),Ŀ���ͼ������(destination_y, destination_x),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
%% line1. ���ó�ʼ��������
background=map;
start_point.x=start_x;
start_point.y=start_y;
% ������ʼ������start_theta��ע��
destination.x=end_x;
destination.y=end_y;
% ShipLong=4;
% Movelength=20;  %����
% SurroundPointsNum=20; %������������n���A*
% valueAPF=2;  %APF�Ƴ��ļ�ֵ����
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %ת��뾶

%��ʼ����
%% line2. ��ʼ׼����
%�������λ���ڵ�ͼ��Χ֮����ߴ���״̬����ȫ���㷨��������ʾ�ް�ȫ·����
%���������ʼ�ڵ������(���ꡢ���򡢳ͷ�ֵ���ƶ�����G����Ŀ����Ԥ�ƴ���H���ܴ���F����һ���ƶ�����r�����ڵ㡢�ӽڵ��)
%�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
if (0<start_point.x<length(background(:,1))&&0<start_point.y<length(background(1,:)))
    start_point.G=0; %�ƶ����� G
    start_point.H=sqrt((destination.x-start_point.x)^2+(destination.y-start_point.y)^2);  %��Ŀ����Ԥ�ƴ���H
    start_point.F=start_point.G+start_point.H; %�ܴ���F
    start_point.R= Movelength; %��һ���ƶ�����r
    start_point.Dir=start_theta;  %��ʼ������
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
    if NodeOpti==1
        %% �㷨4���ڽ��ڵ���ѡ
        %���룺A*�㷨�е�close�б�
        %������Ż����close�б�
        %%%��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        L_Close=length(SetClose);
        ComPoint=[];
        if L_Close>2
            ComPoint=SetClose(end).father;
            while ~(ComPoint.y==start_y && ComPoint.x==start_x)
                if ((SetClose(end).x-ComPoint.x)^2+(SetClose(end).y-ComPoint.y)^2)<(ComPoint.R)^2
                    SetClose(end).father=ComPoint;
                    SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.x,ComPoint.y);
                end
                ComPoint=ComPoint.father;
            end
        end
        SetClose(end).father=ComPoint;
    end
    %% %%�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. ���� FMin �ڵ�ĳͷ�ֵ��С������һ��Ӧ���ƶ��Ĳ��� ShipSpeed��
    ShipSpeed=Movelength * (1-map(SetClose(end).x,SetClose(end).y));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.���㴬���ƶ�һ���ľ������movecost��Ӧ��չ������ڵ���Num��
    movecost=10; %���Ϊ���ٵ�A*������movecost�������ı�
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For ���ɵ�ÿһ������ڵ�Surround(i)��
        Surround(ii).x=floor(SetClose(end).x+ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).y=floor(SetClose(end).y-ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).x,SetClose(end).y,Surround(ii).x,Surround(ii).y);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%���ټ������ڵ������
        if ~isempty( SetOpen)
            openitear=1;
            mindis = 1000;
            while (openitear<length(SetOpen))
                dis=sqrt((Surround(ii).x-SetOpen(openitear).x)^2+(Surround(ii).y-SetOpen(openitear).y)^2);
                if(dis<mindis)
                    mindis=dis;
                    replace=openitear;
                end
                openitear=openitear+1;
            end
            if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).x,Surround(ii).y,SetOpen(replace).x,SetOpen(replace).y,ShipLong/2)==1)
                %                         if (mindis<6)
                Surround(ii).x=SetOpen(replace).x;
                Surround(ii).y=SetOpen(replace).y;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % line10.If(Sourround(i)��Ŀ������Χ֮�ڣ���Sourround(i)�������ϰ����Sourround(i)����close�б���,�Ҵ�FMin�ƶ���Sourround(i)�����д�����ȫ,���˶����̲��ܴ����˶���������)
        if (Surround(ii).x<=0||Surround(ii).x>=length(background(:,1))||Surround(ii).y<=0||Surround(ii).y>=length(background(1,:))...
                || background(Surround(ii).x,Surround(ii).y)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).x,SetClose(end).y,Surround(ii).x,Surround(ii).y,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).x,Surround(ii).y,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. ����Sourround(i)��G��H��Fֵ,����FMinΪSourround(i)�ĸ��ڵ㣻
            Surround(ii).H=sqrt((destination.x-Surround(ii).x)^2+(destination.y-Surround(ii).y)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).x,Surround(ii).y);%movecost���ڵ����Ƴ�����ֵ
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %����FMinΪSourround(i)�ĸ��ڵ㣻
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)�������겻ͬ��open�б������������)
                SetOpen=[SetOpen;Surround(ii)]; %line13. ��Sourround(i)����open�б�
            else %line14
                %% line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
                for kk=1:length(SetOpen)
                    if (Surround(ii).x==SetOpen(kk).x && Surround(ii).y == SetOpen(kk).y)
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

%% ������ȡ
CurrentPoint=destination;
PosTemp=[];
courseTemp=[];
posData0=[];
courseData=[];
while ~(CurrentPoint.x==start_point.x && CurrentPoint.y==start_point.y)
    position=[CurrentPoint.x  CurrentPoint.y];
    PosTemp =[PosTemp;position];
    courseTemp =[courseTemp;CurrentPoint.Dir];
    CurrentPoint = CurrentPoint.father;    
end

%դ������ת��Ϊʵ������
posData0=[posData0;flipud(PosTemp)];
deltaPos=MapSize(1)*ones(size(posData0));
posData=posData0-deltaPos;
courseData=[courseData;flipud(courseTemp)];
courseData_deg=courseData/pi*180;

% AstarData=[posData,courseData_deg];

    Boat(i).AsPos=posData;
    Boat(i).AsCourse=courseData;
    Boat(i).AsCourse_deg=courseData_deg;
    for ii=1:1:length(Boat(i).AsPos)
        
        Boat(i).DecPos(ii,1)=(Boat(i).AsPos(ii,1))*Res-MapSize(1)*1852;
        Boat(i).DecPos(ii,2)=(Boat(i).AsPos(ii,2))*Res-MapSize(2)*1852;
        
    end
end

% % A*����·��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc
disp(['��������ʱ��: ',num2str(toc)]);
