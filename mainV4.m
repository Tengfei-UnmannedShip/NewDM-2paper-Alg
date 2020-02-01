%% (2.0�汾)��չ������Ϊ��λ�ĵ�ͼ
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
    
    % APF��������
    Boat_eta=0.2;
    Boat(i).APF_factor(1)=Boat_eta;
    Boat_alfa=0.01;
    Boat(i).APF_factor(2)=Boat_alfa;
    BoatRiskFieldPeakValue=200;   %�������ֵ���Ը�����Ҫ��������
    Boat(i).APF_factor(3)=BoatRiskFieldPeakValue;
    Boat_Speed_Factor=1;        %�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��
    Boat(i).APF_factor(4)=Boat_Speed_Factor;
    
end

%% ���Ƶ�ǰÿ�Ҵ���APF
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).pos(1,1);
    Boat_y = Boat(i).pos(1,2);
    Boat_theta = -Boat(i).COG_rad; %�˴�Ϊ������
    Boat_Speed = Boat(i).speed;
    APF_factor = Boat(i).APF_factor;
    Boat(i).APF=DrawAPF(Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize*1852,APF_factor,Res);
    
    SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2 );
end

%% APF��ͼ���Գ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

APF=zeros(m,n);
for i=1:1:Boat_Num
    APF=APF+Boat(i).APF;
end

figure
kk1=mesh(X,Y,APF);
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852 0 250])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
box off;

figure
kk2=contourf(X,Y,APF);  %�������ɫ�ĵȸ���ͼ
colorpan=ColorPanSet(6);
colormap(colorpan);%����ɫ��
% axis([xmin xmax ymin ymax zmin zmax])
axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
title(['t=',num2str(tMax),'s'], 'Fontname', 'Times New Roman');
box on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% A*�㷨��ʼ
% % valueAPF0=[0.1,0.5,1,2,5,10,50,100];
%
% for i=1:1:1
%     start_x = Boat(i).pos(1,1)+MapSize(1)*1852;
%     start_y = Boat(i).pos(1,2)+MapSize(2)*1852;
%     start_theta = Boat(i).COG_rad;   %��ʼ�����򣬻�����
%     %A*�㷨��Ŀ��㣬��ÿ��ʱ�̣����·�����Ӧ��������·���㣬����·����󣬲���������յ�Ŀ���
%     end_x = Boat(i).goal(1,1)+MapSize(1)*1852;
%     end_y = Boat(i).goal(1,2)+MapSize(2)*1852;
%
%     Astar_map=zeros(m,n);
%     for k=1:1:Boat_Num
%         if k~=i
%             Astar_map=Astar_map+Boat(k).APF;
%         end
%     end
%     ShipLong=ShipSize(i,1);
%     Movelength=Boat(i).speed;  %����,��ǰʱ�̵��ٶ�
%     SurroundPointsNum=20; %������������n���A*
%     valueAPF=2;  %APF�Ƴ��ļ�ֵ����
%     NodeOpti=0;
%     [posData,courseData,courseData_deg] = AstarMain(Astar_map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
%
%     Boat(i).AsPos=posData;
%     Boat(i).AsCourse=courseData;
%     Boat(i).AsCourse_deg=courseData_deg;
%
% end
% % A*����·��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Astar_map=zeros(m,n);
% for i=1:1:4
%     Astar_map=Astar_map+Boat(i).APF;
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
