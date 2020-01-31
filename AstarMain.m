function [posData,courseData,courseData_deg] = AstarMain(map,start_x,start_y,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize)
%% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_y,start_x),Ŀ���ͼ������(destination_y, destination_x),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
%% line1. ���ó�ʼ��������
background=map;
start_point.x=start_x;
start_point.y=start_y;
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
courseData_deg=courseData/pi*180+180;

% AstarData=[posData,courseData_deg];
end

