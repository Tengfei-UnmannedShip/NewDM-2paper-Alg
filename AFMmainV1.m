%% (4.25�汾)���㵱ǰCAL�µ��������̣���ǰΪ׷Խ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.(1.0�汾)FM��APF��ϵĵ����棬FM��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
%   (2.5�汾)��WangNing��ShipDomain����ԭAPF����չ������Ϊ��λ�ĵ�ͼ
% 2.(3.0�汾)���·����
%   (3.5�汾)FM�㷨ֻ����δ��5����10����
%       ����1:ֻ����10������¼�����е�ʱ��ͽ��������ȫ��������Աȣ�ȷ��ֻ����10����׼ȷ�Ժ�Ч�ʣ�
%       ����2:����·���㣬Ȼ���һ�ξ��߼���10����Ȼ���ڵ�һ��forѭ����ʼʱ����״̬��ÿ5��������������5���ӣ�
%            t=60*5��������һ��·���㣬���û��·�����ˣ������յ�ΪĿ�꣬�������Ĺ����У�����ٴγ���·���㣬
%            ������·����ΪĿ�ꣻ
%   (3.7�汾)���õ���·���㣬���ٲ����ۺ�·����
%       ����1:�����ҵ��з��յ�Ŀ�괬���ҵ����з��յ��Ǹ���Ȼ���ҵ�����Ӧ��·����
%       ����2:һ�Ҵ�һ�Ҵ���·��
% 3.(4.0�汾)·���滮�㷨����FM�㷨
%   (4.1�汾)ֻ����1�Ҵ���1����
%       ����1��Ŀ���Ϊ·���㣻
%       ����2��Ŀ���Ϊ·���㣬ʶ���Ѿ�����·�����Ŀ����Ϊ�յ�
%   (4.2�汾)���㵱ǰCAL�µ��������̣���¼ʱ�䣻
%       ����1������ʲôʱ�����ʲô����·���㣬ֻҪ�ܹ���ͨ�����ǿ��Գɹ�������
%       ����2������CAL�����ã���ǰ�ǹ̶�CAL���ڶ�̬CAL�£�����ʲô���أ�
%    ���հ�ΪFMmainV25.m�����Գɹ����У���������û�к���Ƕȵ����ƣ����¹滮���ĺ���������
% 4.(5.0�汾)ʹ��AFM�㷨�������ϱ�����ǰʱ�̲��ɴ�λ�õ�����
%   (5.1�汾)����AFM
%       ����1�������򵥵�ֱ�ߵ�δ���������������
%       ����2������OE�����л���������NOMOTO�㷨����������ϸ������
%       ����3�����Ը���NOMOTO�㷨��������Ч���ٶ����Ǳ仯������
%   (5.2�汾)��·�����Ϊ���򳡣����ϲ��Ʋ��CAL
%       ����1����TRB�����еĹ���Ӧ�ý��������չ̶���CAL������
%       ����2�������ĺ��г���ÿ�Ҵ��ķ��ճ���ÿ�Ҵ����е�ǰ��CAL���Ĵ��������
% 5.(6.0�汾)����CAL�Ʋ⣬�Ŵ󾫶ȣ����ؿ��巽�������ε�Ԥ�����������б�Ҷ˹����
%       ����1�����ݼ�ⷶΧ�ͷ��մ�����Ŀȷ�������������ݲ�ͬ�ĳ��������Ʋ��TS���еĳ�����
%             �����Ʋ��TS�ĺ��з����������ؿ�����棬Ϊ�˱�֤�ٶȣ����ȿ�������200m��185.2m,��������
%       ����2��
%       ����3��
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




clear
clc
close all

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
CAL0=[
    2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

Boat_Num=4;   %��������
tMax=3600;    %���ʱ��
tBayes=2000;  %��Ҷ˹�Ʋ�ʱ��
t_count11=0;    %ʱ�����
t_count12=0;    %ʱ�����
t_count21=0;    %ʱ�����
t_count22=0;    %ʱ�����
Start_pos=[];
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
Res=50;  %Resolution��ͼ�ķֱ���
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
    
end

for t=1:1:2500
    t_count11=t_count12;    %ʱ�����
    reach_label=0; %ÿ��ʱ�̹���
    %% ÿ��ʱ�̵�״̬����
    for i=1:1:Boat_Num
        if Boat(i).FM_lable~=0 && Boat(i).reach==1 %���Ѿ���ʼ���ߵ�û�е��յ㣬��ʱ���վ�����
            pos_temp=0;
            row=Boat(i).Current_row;
            while pos_temp<Boat(i).speed
                
                delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                delta_pos=norm(delta_pos0);
                pos_temp=pos_temp+delta_pos;
                row=row+1;
                
            end
            detaPos=Boat(i).path(row,:)-Boat(i).pos;
            Boat(i).pos = Boat(i).path(row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = NavAng(detaPos);
            Boat(i).COG_rad = Boat(i).COG_deg/180*pi;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            Boat(i).Current_row=row;
            
        elseif Boat(i).reach==0  %�Ѿ������յ�
            Boat(i).pos = Boat(i).pos;
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).COG_deg;
            Boat(i).COG_rad = Boat(i).COG_rad;
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
        elseif Boat(i).FM_lable==0  %û�о��߹���״̬
            
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=2*Res %������ǰ������ͬһ�����������Ϊ��������Ŀ���
            disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����Ŀ���']);
            Boat(i).reach=0;
        end
        reach_label=reach_label+Boat(i).reach;
    end
    if reach_label<=1    %û���յ�ʱΪ1������Ϊ0����ˣ�����3�Ҵ������յ��reach_label<=1
        disp([num2str(t),'ʱ��','���д���ɱ������������']);
        break
    end
    
    for OS=1:1:1    %Boat_Num
        %�жϵ�ǰiʱ������OS���ľ���������,compliance==1����������,��δ����Ŀ���
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(i).reach==1
            disp([num2str(t),'ʱ��',num2str(OS),'����ʼ����']);
            
            %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
            if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
                % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
                Boat(OS).CAL=CAL0(OS,:);
            elseif  shipLabel(OS,2)==1
                % ��Ҷ˹�Ʋ�
                Boat(OS).CAL=CAL0(OS,:); %��Ҷ˹�ƶ����ó��ģ����ǵ�ǰʱ�̵�Boat(i).CAL
            end
            %% ����·���滮������
            % �ڵ�ǰʱ�����¼���FM������������
            %   1.֮ǰ��FM�������Ѿ��þ�����Ҫ���¼��㣻
            if Boat(OS).FM_lable>=size(Boat(OS).FMPos,1)
                Calculate_lable=1;
                
            else
                Calculate_lable=0;
            end
            
            if Calculate_lable==1
                %% ���������ĺ��г�
                % ���Ƶ�ǰ��������Ŀ�괬��SCR
                t_count31=toc;    %ʱ�����
                                        SCR_temp=zeros(m,n);
                        CAL_Field=zeros(m,n);
                        ScenarioMap=zeros(m,n);
                PeakValue=100;
                for TS=1:1:Boat_Num
                    if TS~=OS

                        Boat_x = Boat(TS).pos(end,1);
                        Boat_y = Boat(TS).pos(end,2);
                        Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(TS,1);
                        
                        SCR_temp= ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,PeakValue,2);
                        
                        %������������µķ��ճ�������RuleField
                        CAL=Boat(OS).CAL(TS);
                        CAL_Field= RuleField( Boat_x,Boat_y,Boat_theta,Shiplength,MapSize,Res,PeakValue,CAL);
                        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
                        
                    end
                end

                
                % ���Ƶ�ǰ�����ĺ�������
                Boat_x=Boat(OS).pos(1,1);
                Boat_y=Boat(OS).pos(1,2);
                Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
                Shiplength = ShipSize(OS,1);
                alpha=30;
                AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,Shiplength,MapSize,Res,200);
                ScenarioMap=ScenarioMap+AFMfiled;
                
                %     %% ��ͼ����
                %     figure;
                %     kk1=mesh(X,Y,Scenario);
                %     colorpan=ColorPanSet(6);
                %     colormap(colorpan);%����ɫ��
                %     hold on
                %     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %     hold on;
                %     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
                %     axis equal
                %     axis off
                %     %     surf(X,Y,APFValue);
                %
                %     figure
                %     % kk2=pcolor(APFValue);
                %     kk2=contourf(X,Y,Scenario);  %�������ɫ�ĵȸ���ͼ
                %     colorpan=ColorPanSet(6);
                %     colormap(colorpan);%����ɫ��
                %     % set(kk2, 'LineStyle','none');
                %     hold on
                %     plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %     hold on
                %     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );
                %     % axis equal
                %     % axis off
                FM_map=1./ScenarioMap;
                t_count32=toc;
                disp([num2str(OS),'�Ŵ����㺽�г���ʱ: ',num2str(t_count32-t_count31)]);
                %% FM�㷨������
                start_point(1,1) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                start_point(1,2) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                
                end_point(1,1) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
                end_point(1,2) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
                
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
                [Mtotal, paths] = FMM(FM_map, end_point', start_point');
                Boat(OS).FM_lable=Boat(OS).FM_lable+1;
                path0 = paths{:};
                path0 =path0';
                
                posData = zeros(size(path0));
                posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                Boat(OS).path=posData;
                Boat(OS).Current_row=1;   %ÿ�����¾��ߣ���ǰ������1
                t_count22=toc;
                disp([num2str(OS),'�Ŵ����������ʱ��: ',num2str(t_count22-t_count21)]);
            end
        end
    end
    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵����д���������ʱ��: ',num2str(t_count12-t_count11)]);
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);
