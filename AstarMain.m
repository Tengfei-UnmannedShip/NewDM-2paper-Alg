function [posData,courseData,courseData_deg] = AstarMain(map,start_x,start_y,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize)
%% 算法5:多向A*算法
%输入: 惩罚图像(矩阵)map,起点图像坐标(start_y,start_x),目标点图像坐标(destination_y, destination_x),船舶长度ShipLong,旋回半斤Rmin
%输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表
%% line1. 设置初始船舶艏向；
background=map;
start_point.x=start_x;
start_point.y=start_y;
destination.x=end_x;
destination.y=end_y;
% ShipLong=4;
% Movelength=20;  %步长
% SurroundPointsNum=20; %跳整方向数，n向的A*
% valueAPF=2;  %APF势场的价值函数
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %转弯半径

%开始计算
%% line2. 初始准备：
%如果船舶位置在地图范围之外或者船舶状态不安全，算法结束，提示无安全路径，
%否则计算起始节点各属性(坐标、艏向、惩罚值、移动代价G、到目标点的预计代价H、总代价F、下一步移动距离r、父节点、子节点等)
%并将该节点放到open表中,初始化close列表为空；
if (0<start_point.x<length(background(:,1))&&0<start_point.y<length(background(1,:)))
    start_point.G=0; %移动代价 G
    start_point.H=sqrt((destination.x-start_point.x)^2+(destination.y-start_point.y)^2);  %到目标点的预计代价H
    start_point.F=start_point.G+start_point.H; %总代价F
    start_point.R= Movelength; %下一步移动距离r
    start_point.Dir=pi/2;  %起始点艏向
    SetOpen(1)=start_point; %起始点坐标
    SetOpen(1).father=nan; %父节点
    SetClose(1)=SetOpen(1); %并将该节点放到open表中,初始化close列表为空；
end
%% 开始计算
while  ~isempty(SetOpen)  %line3.While: open 列表不为空
    for ii=2:length(SetOpen)  %line4.寻找open列表中F值最小的节点，记为FMin；
        if SetOpen(ii).F < SetOpen(1).F
            a=SetOpen(ii);
            SetOpen(ii)=SetOpen(1);
            SetOpen(1)=a;
        end
    end
    SetClose=[SetClose;SetOpen(1)]; %line5-1.将FMin加入close列表,所以FMin就是SetClose(end),同时在open列表中删除该点；
    SetOpen(1)=[]; %line5-2.将FMin加入close列表，同时在open列表中删除该点；
    Surround=[];
    if NodeOpti==1
        %% 算法4：邻近节点优选
        %输入：A*算法中的close列表
        %输出：优化后的close列表
        %%%回馈处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    %% %%变步长设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. 根据 FMin 节点的惩罚值大小计算下一步应该移动的步长 ShipSpeed；
    ShipSpeed=Movelength * (1-map(SetClose(end).x,SetClose(end).y));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.计算船舶移动一步的距离代价movecost和应扩展的邻域节点数Num；
    movecost=10; %如果为变速的A*，所以movecost在这里，会改变
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For 生成的每一个邻域节点Surround(i)；
        Surround(ii).x=floor(SetClose(end).x+ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).y=floor(SetClose(end).y-ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).x,SetClose(end).y,Surround(ii).x,Surround(ii).y);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%不再计算相邻点的条件
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
        % line10.If(Sourround(i)在目标区域范围之内，且Sourround(i)处不是障碍物，且Sourround(i)不在close列表中,且从FMin移动到Sourround(i)过程中船舶安全,且运动过程不受船舶运动规律限制)
        if (Surround(ii).x<=0||Surround(ii).x>=length(background(:,1))||Surround(ii).y<=0||Surround(ii).y>=length(background(1,:))...
                || background(Surround(ii).x,Surround(ii).y)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).x,SetClose(end).y,Surround(ii).x,Surround(ii).y,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).x,Surround(ii).y,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. 计算Sourround(i)的G、H、F值,设置FMin为Sourround(i)的父节点；
            Surround(ii).H=sqrt((destination.x-Surround(ii).x)^2+(destination.y-Surround(ii).y)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).x,Surround(ii).y);%movecost用于调整势场代价值
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %设置FMin为Sourround(i)的父节点；
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)所在坐标不同于open列表中任意点坐标)
                SetOpen=[SetOpen;Surround(ii)]; %line13. 将Sourround(i)加入open列表；
            else %line14
                %% line15.比较Sourround(i)与open列表中具有相同坐标节点的G值，设置较小者的父节点为FMin；
                for kk=1:length(SetOpen)
                    if (Surround(ii).x==SetOpen(kk).x && Surround(ii).y == SetOpen(kk).y)
                        rember=kk;                       %找到Sourround(i)与open列表中具有相同坐标的节点
                    end
                end
                if Surround(ii).G < SetOpen(rember).G     %比较G值
                    SetOpen(rember).father=SetClose(end); %设置较小者的父节点为FMin；
                end
            end %line16.
        end     %line17.
    end         %line18.
    if SetClose(end).H < ShipSpeed %line19. 如果FMin到目标点的距离小于移动步长，算法结束；
        break;
    end
end
destination.father=SetClose(end);
destination.Dir=ShipDirection(SetClose(end).x,SetClose(end).y,end_x,end_y);

%% 参数提取
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

%栅格坐标转换为实际坐标
posData0=[posData0;flipud(PosTemp)];
deltaPos=MapSize(1)*ones(size(posData0));
posData=posData0-deltaPos;
courseData=[courseData;flipud(courseTemp)];
courseData_deg=courseData/pi*180+180;

% AstarData=[posData,courseData_deg];
end

