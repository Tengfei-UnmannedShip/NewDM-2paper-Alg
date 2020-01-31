%% A*��APF��ϵĵڶ��棬��׼��ǰλ����Ŀ���
%%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
clear all;
clc;

[X,Y]=meshgrid(-300:1:300,-300:1:300);  
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
Boat.State(4,:)=[ 120  -150   -45    10  20   5]; %ship4�Ǳ���
%Boat.State(5,:)=[-70   -15    90    10  20   5];
goal=[-280  250];%��ʱ�����ڣ�120��-150���Ƕ�-45
k_near=10;%����Ŀ��㸽��������Ҫ������ϵ��
k_far=10; %���������ż�֮�������㶨ʱ������ϵ����һ����k_near��ͬ���������ż��������������ͻ��
Po_att=100;%�����ż��뾶
k=0.3;%����ϵ��
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);  
% U_att=0.5*k_near*d_obs_APF.^2;%���������ż���

% U_att=0.5*k_near*d_goal.^2.*(d_goal<=Po_att)+k_far*Po_att*d_goal.*(d_goal>Po_att);%�������ż���
% U_att=d_goal.*k ;  %����Ϊ�����ģ��������Ǿ����һ�κ���

for i=1:1:Boat_Num
    %�ֲ�����ϵת��
        
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

%% ��ͼ
figure
    mesh(X,Y,APFValue);
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );
    axis equal;
    axis off;
%     surf(X,Y,APFValue);
figure
    contourf(X,Y,APFValue,'LevelStep',30);  %�������ɫ�ĵȸ���ͼ
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    hold on;
    ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );

%     plot(goal(1,1),goal(1,2),'rx');
    axis equal;
    axis off;

figure
%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
[SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100); %��������CircleAStar(map,start_row,start_col,end_row,end_col)