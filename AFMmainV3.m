%% (5.2�汾)����AFM��ֱ�����ʼ�ͽ�ʵ������ת��Ϊդ�����꣬�Ժ����е������ͼ������դ�����꣬����18.52Ϊ��λ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.(1.0�汾)FM��APF��ϵĵ����棬FM��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
%   (2.5�汾)��WangNing��ShipDomain����ԭAPF����չ������Ϊ��λ�ĵ�ͼ
% 2.(3.0�汾)����·����
%   (3.5�汾)A*�㷨ֻ����δ��5����10����
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
%       ����3��ֱ�����ʼ�ͽ�ʵ������ת��Ϊդ�����꣬�Ժ����е������ͼ������դ�����꣬����18.52Ϊ��λ
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
% ��ʼ���������д���ȫ������
% ����CAL�Ĺ涨��ÿ��ʹ�õĶ���һ�У�����OS��TS��CAL��
% ����CAL0(1,2)=0����˼�Ǳ�����TS��̬����0��������ʲô����������stand-on ship��OSҪ��TS�Ĵ�ͷ��
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

%��ͼ����
% MapSize_temp=max(max(abs(Start_pos)))/1852;
MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52*2;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %�����־������Ŀ���ʱΪ0��δ����ʱΪ1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    Boat(i).HisCOG=[Boat(i).COG_rad,Boat(i).COG_deg];
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    pos0=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    %�ѳ�ʼλ�ù�һ��դ��λ���ϣ�������Ȼ������
    Boat(i).pos(1,1) = round(pos0(1,1)/Res)*Res;
    Boat(i).pos(1,2) = round(pos0(1,2)/Res)*Res;
    Boat(i).HisPos=Boat(i).pos;
    %��һ�ξ��ߵ�������
    Boat(i).path = [];
    Boat(i).RiskData=[];
end


%�������߲�������
for i=1:1:Boat_Num
    %��Ŀ���λ�ù�һ��դ��λ���ϣ�������Ȼ������
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).decision_count=0; %��ʼʱ��decision_countΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).Current_row=0;
    Boat(i).RiskHis=[];
    Boat(i).End=[];         %��¼Σ�մ�����ʷ
    
end

for t=1:1:3600*2
    t_count11=t_count12;    %ʱ�����
    reach_label=0; %ÿ��ʱ�̹���
    %% ÿ��ʱ�̵�״̬����
    for i=1:1:Boat_Num
        row=0;
        if Boat(i).FM_lable~=0 && Boat(i).reach==1 %���Ѿ���ʼ���ߵ�û�е��յ㣬��ʱ���վ�����
            pos_temp=0;
            row=Boat(i).Current_row;
            
            while pos_temp<ceil(Boat(i).speed/Res)
                
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
    if  Boat(1).reach==0
        disp([num2str(t),'ʱ��','1����ɱ������������']);
        break
    elseif Boat(2).reach==0
        disp([num2str(t),'ʱ��','2����ɱ������������']);
        break
    elseif Boat(3).reach==0
        disp([num2str(t),'ʱ��','3����ɱ������������']);
        break
    elseif Boat(4).reach==0
        disp([num2str(t),'ʱ��','4����ɱ������������']);
        break
    end
    if reach_label<=1    %û���յ�ʱΪ1������Ϊ0����ˣ�����3�Ҵ������յ��reach_label<=1
        disp([num2str(t),'ʱ��','���д���ɱ������������']);
        break
    end
    
    for OS=1:1:Boat_Num    %Boat_Num
        %�жϵ�ǰiʱ������OS���ľ���������,compliance==1����������,��δ����Ŀ���
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(OS).reach==1
            
            if Boat(OS).decision_count<=round(20/ShipInfo(OS,5)) &&  Boat(OS).decision_count>0 ...    %ÿ�ξ���֮�󱣳�20s
                    && Boat(OS).Current_row <= size(Boat(OS).path,1)-10         %��Ԥ���ߵ�pathֻʣ����50����ʱ��ǿ�����¼���
                
                disp([num2str(t),'ʱ��',num2str(OS),'����',num2str(Boat(OS).decision_count),'���ڱ������־���']);
                Boat(OS).decision_count=Boat(OS).decision_count+1;
                
            else
                disp([num2str(t),'ʱ��',num2str(OS),'����ʼ����']);
                
                %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
                if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
                    % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
                    Boat(OS).CAL=CAL0(OS,:);
                elseif  shipLabel(OS,2)==1
                    % ��Ҷ˹�Ʋ�
                    Boat(OS).CAL=CAL0(OS,:); %��Ҷ˹�ƶ����ó��ģ����ǵ�ǰʱ�̵�Boat(i).CAL
                end
                
                %% ���������ĺ��г�
                % ���Ƶ�ǰ��������Ŀ�괬��SCR
                t_count31=toc;    %ʱ�����
                SCR_temp=zeros(m,n);
                CAL_Field=zeros(m,n);
                ScenarioMap=zeros(m,n);
                AFMfiled=zeros(m,n);
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
                
                % ���Ƶ�ǰ�����ĺ�������
                Boat_x=Boat(OS).pos(1,1);
                Boat_y=Boat(OS).pos(1,2);
                Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
                Shiplength = ShipSize(OS,1);
                alpha=30;
                R=500;
                AFMfiled=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                ScenarioMap=1+ScenarioMap+AFMfiled;
                
                %                 %% ��ͼ����
                %                 figure;
                %                 kk1=mesh(X,Y,ScenarioMap);
                %                 colorpan=ColorPanSet(6);
                %                 colormap(colorpan);%����ɫ��
                %                 hold on
                %                 plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %                 hold on;
                %                 ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5), ShipInfo(OS,6), ShipInfo(OS,3),1 );
                %                 axis equal
                %                 axis off
                %
                %                 figure
                %                 kk2=contourf(X,Y,ScenarioMap);  %�������ɫ�ĵȸ���ͼ
                %                 colorpan=ColorPanSet(6);
                %                 colormap(colorpan);%����ɫ��
                %                 % set(kk2, 'LineStyle','none');
                %                 hold on
                %                 plot(Boat(OS).goal(1,1),Boat(OS).goal(1,2),'ro','MarkerFaceColor','r');
                %                 hold on
                %                 %                     ship_icon(ShipInfo(OS,1),ShipInfo(OS,2),ShipInfo(OS,5),ShipInfo(OS,6), ShipInfo(OS,3),1 );
                
                FM_map=1./ScenarioMap;
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
                start_point(1,1) = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                start_point(1,2) = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                
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
                        
                        end_point(1,1) =round((WP(1,1)+MapSize(1)*1852)/Res);
                        end_point(1,2) =round((WP(1,2)+MapSize(2)*1852)/Res);
                        
                    end
                end
                
                Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];
                
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
                    [Mtotal, paths] = FMM(FM_map, end_point', start_point');
                    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
                    path0 = paths{:};
                    path0 =path0';
                    
                    posData = zeros(size(path0));
                    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                    
                    end_point(1)=end_point(1)*Res-MapSize(1)*1852;
                    end_point(2)=end_point(2)*Res-MapSize(2)*1852;
                    Boat(OS).path=posData;
                    Boat(OS).Current_row=1;   %ÿ�����¾��ߣ���ǰ������1
                    Boat(OS).decision_count=1;
                    
                end
            end
        end
    end
    
    
    if t==1000
        Boat1000=Boat;
        t1000=t_count12;
        save('data0226-1500','Boat1000','t1000');
        disp('1000s�����ѱ���');
        clear Boat1000
    elseif t==1500
        Boat1500=Boat;
        t1500=t_count12;
        save('data0226-1500','Boat1500','t1500','-append');
        disp('1500s�����ѱ���');
        clear Boat1500
    elseif t==2000
        Boat2000=Boat;
        t2000=t_count12;
        save('data0226-1500','Boat2000','t2000','-append');
        disp('2000s�����ѱ���');
        clear Boat2000
    elseif t==2500
        Boat2500=Boat;
        t2500=t_count12;
        save('data0226-1500','Boat2500','t2500','-append');
        disp('2500s�����ѱ���');
        clear Boat2500
    end
    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵����д���������ʱ��: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);