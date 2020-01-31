%% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.A*��APF��ϵĵ����棬A*��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
% 2.(3.0�汾)���·����
% 3.(4.0�汾)��ӱ�Ҷ˹


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

Boat_Num=4;%��������
%�۲���λ��
Boat(1).State(1,:)=[   0   220   180     5  30  10];  %����״̬���꣨x,y,ang,v,l,w��angΪ�������� vΪ�ٶȴ�С,lΪ������wΪ����
Boat(2).State(1,:)=[-200    50   120     5  20   6];  %����״̬���꣨x,y,ang,v��angΪ��������
Boat(3).State(1,:)=[ 150    70  -120     8  10   3];
Boat(4).State(1,:)=[ 120  -150   -45    10  20   5];  %ship4�Ǳ���

for i=1:1:Boat_Num
    
    Boat(i).GoalRange=MapSize-[15,15];
    Boat(i).goal=Goal_point(Boat(i).State(1,1),Boat(i).State(1,2),Boat(i).State(1,3),Boat(i).GoalRange);
    
    Boat_eta=1;
    Boat(i).APF_factor(1)=Boat_eta;
    Boat_alfa=0.1;
    Boat(i).APF_factor(2)=Boat_alfa;
    BoatRiskFieldPeakValue=100;   %�������ֵ���Ը�����Ҫ��������
    Boat(i).APF_factor(3)=BoatRiskFieldPeakValue;
    Boat_Speed_Factor=1;        %�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��
    Boat(i).APF_factor(4)=Boat_Speed_Factor;
    
end
for i=1:1:Boat_Num
    
    Boat_x = Boat(i).State(1,1);
    Boat_y = Boat(i).State(1,2);
    Boat_theta = -Boat(i).State(1,3)/180*pi;
    Boat_Speed = Boat(i).State(1,4);
    APF_factor = Boat(i).APF_factor;
    Boat(i).APF=DrawAPF(Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize,APF_factor);
    
end

% %% APF��ͼ���Գ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% APF=zeros(m,n);
% for i=1:1:length(Boat)
% APF=APF+Boat(i).APF;
%
% subplot(4,2,2*i-1)
% kk1=mesh(X,Y,Boat(i).APF);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% hold on
% plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
%
% subplot(4,2,2*i)
% kk2=contourf(X,Y,Boat(i).APF);  %�������ɫ�ĵȸ���ͼ
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% hold on
% plot(Boat(i).goal(1,1),Boat(i).goal(1,2),'ro','MarkerFaceColor','r');
%
% end
%
% figure;
% subplot(1,2,1)
% kk1=mesh(X,Y,APF);
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
%
% subplot(1,2,2)
% kk2=contourf(X,Y,APF);  %�������ɫ�ĵȸ���ͼ
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% A*�㷨��ʼ
for i=1:1:4
    start_x = Boat(i).State(1,1)+MapSize(1);
    start_y = Boat(i).State(1,2)+MapSize(2);
    end_x = Boat(i).goal(1,1)+MapSize(1);
    end_y = Boat(i).goal(1,2)+MapSize(2);
    Astar_map=zeros(m,n);
    for k=1:1:Boat_Num
        if k~=i
            Astar_map=Astar_map++Boat(k).APF;
        end
    end
    ShipLong=4;
    Movelength=20;  %����
    SurroundPointsNum=20; %������������n���A*
    valueAPF=2;  %APF�Ƴ��ļ�ֵ����
    NodeOpti=0;
    [posData,courseData,courseData_deg] = AstarMain(Astar_map,start_x,start_y,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize);
    Boat(i).AsPos=posData;
    Boat(i).AsCourse=courseData;
    Boat(i).AsCourse_deg=courseData_deg;
end

%% A*����·��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% kk2=contourf(X,Y,Astar_map);  %�������ɫ�ĵȸ���ͼ
% colorpan=ColorPanSet(6);
% colormap(colorpan);%����ɫ��
% hold on
% plot(Boat(1).goal(1,1),Boat(1).goal(1,2),'ro','MarkerFaceColor','r');
% hold on;
% ship_icon(Boat(1).State(1,1),Boat(1).State(1,2),Boat(1).State(1,5), Boat(1).State(1,6), Boat(1).State(1,3),2 );
% 
% AstarCourse=Boat(1).Astar_course-180;
% 
% for i=1:1:length(Boat(1).Astar_pos)
%     hold on;
%     ship_icon(Boat(1).Astar_pos(i,1),Boat(1).Astar_pos(i,2),Boat(1).State(1,5)/5, Boat(1).State(1,6)/5, AstarCourse(i),1 );
% end
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc
disp(['��������ʱ��: ',num2str(toc)]);
