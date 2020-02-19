%% (4.2�汾)���㵱ǰCAL�µ��������̣���ǰΪ��������
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
% 4.(5.0�汾)���뱴Ҷ˹�ƶϵ�����



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
ShipInfo=[
    0.0, 0.0,  18,    0,    3,  6
    0.0, 0.0,  18,  230,    4,  6
    0.0, 0.0,  16,  300,    5,  6
    0.0, 0.0,  13,  135,    5,  6
    ];

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
GoalRange=MapSize-[0.5,0.5];
Res=10;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%�������߲�������
for i=1:1:Boat_Num
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    
    Boat(i).WayPoint_temp = [];
    Boat(i).WayPoint = [];
    Boat(i).WP_data = [];
    Boat(i).HisWP = [];
    
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).FMPos=[];
    Boat(i).FMCourse=[];
    Boat(i).FMCourse_deg=[];
    
end

for t=1:1:tMax
    t_count11=t_count12;    %ʱ�����
    
    %% ÿ��ʱ�̵�״̬����
    for i=1:1:Boat_Num
        if Boat(i).FM_lable~=0  %���Ѿ���ʼ�����ˣ���ʱ���վ�����
            pos_temp=0;
            row=Boat(i).Current_row;
            while pos_temp>Boat(i).speed
                
                delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                delta_pos=norm(delta_pos0);
                pos_temp=pos_temp+delta_pos;
                row=row+1;
                
            end
            Boat(i).Current_row=row;
            Boat(i).pos = Boat(i).path(Current_row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).FMCourse_deg(Current_row,:);
            Boat(i).COG_rad = Boat(i).FMCourse(Current_row,:);
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
            
        else  %û�о��߹���״̬
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=Res %������ǰ������ͬһ�����������Ϊ��������Ŀ���
            disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����Ŀ���']);
            Boat(i).reach=0;
        end
    end
    if Boat(1).reach==0 && Boat(2).reach==0 && ...
            Boat(3).reach==0 && Boat(4).reach==0
        disp([num2str(t),'ʱ��','���д�����Ŀ��㣬�������']);
        break
    end
    
    for OS=1:1:Boat_Num
        
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 %�жϵ�ǰiʱ������j���ľ�����������compliance==1����������
            disp([num2str(t),'ʱ��',num2str(OS),'����ʼ����']);
            t_count21=t_count22;    %ʱ�����
            %% ·�������
            % =========================================================================
            % ���룺��ǰ��ÿ�Ҵ���λ��
            % �����ÿһ�ұ������е�ÿһ��Ŀ�괬��·����WP��
            % �������裺
            % 1.����ÿһ�Ҵ����Ĵ�ͷ��β������waypoint(WP),��ŷֱ�Ϊ0(��ͷ��)��1(��β��);
            % 2.�����ҵ����з��յ�Ŀ�괬�����ҵ����з��յ��Ǹ���Ȼ���ҵ�����Ӧ��·����
            % =========================================================================
            WP_label=[];
            WayPoint_temp0=[];
            Risk_temp=[];
            k=1;
            for TS=1:1:Boat_Num
                if TS~=OS
                    v_os = Boat(OS).speed(end,:);
                    course_os = Boat(OS).COG_deg(end,:);
                    pos_os = Boat(OS).pos(end,:);
                    v_ts = Boat(TS).speed(end,:);
                    course_ts = Boat(TS).COG_deg(end,:);
                    pos_ts = Boat(TS).pos(end,:);
                    TSlength = ShipSize(TS,1);
                    d_thre = 1*1852;  %d_threΪ�ж���ײ���յķ�����ֵ
                    changeLabel = 0;
                    % =========================================================================
                    % ���ж�����ײ���գ�����Ҫ����waypoint��
                    % �жϱ�׼���Ƿ�����ײ����
                    % ע�⣺���ܳ���ԭ�������б���Ӧ���Ѿ���ȥ������Ԥ��·������ڹ���Ĵ�ͷһ������
                    % ��ʱ�Ʋ���������������ݲ���A*����������ݵ�ԭ���������һ�ԣ���Ҫ��������
                    % =========================================================================
                    CurrentRisk=CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre);
                    if CurrentRisk==1  %����ȷ����ײ����
                        WP_label(k) = TS;
                        % ��ʼ����waypiont
                        CPA_temp=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                        
                        %���DCPA_temp=0�����������0��Ϊ���������£������С��DCPA����Ϊ1�����̫С�Ļ��������ܴ�Ӱ��ǰ��Ĳ���
                        DCPA_temp=CPA_temp(5);
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
                        
                        %���㵱ǰ��Ŀ�괬�����޷��ա�TCPA��DCPA��������ľ���
                        Risk_temp(k,:) = [TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp];
                        WayPoint_temp0 = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
                        WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0�Ǵ�ͷ��
                        WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1�Ǵ�β��
                        k=k+1;
                    end
                end
            end
            WP_Num = length(WP_label);
            TCPA_OS=sum(Risk_temp(:,3));
            DCPA_OS=sum(Risk_temp(:,4));
            Dis_OS=sum(Risk_temp(:,5));
            Risk_OS=Risk_temp;
            
            Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
            Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
            Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
            %�����Ŀ�����ҵ���������ֵ�����õķ������ۺϷ���TCPA����ȣ�Ȼ��Dis��Ȼ����DCPA��
            %��100��10��1��Ϊϵ�����ֿ������ǲ�֪��Ч����Σ���Ҫ��һ���ĵ���
            Risk_value=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
            
            kk = 1;
            WayPoint_temp1 = [];
            for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %��WP_Num�ҷ��մ�����2^WP_Num������������WP_Numλ
                CAL_temp = dec2bin(scenario);
                for ts_i=1:1:WP_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
                    CAL_ts=str2num(CAL_temp(ts_i+1));
                    if CAL_ts==0
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                    elseif CAL_ts==1
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP1;
                    end
                end
                switch WP_Num
                    case 1
                        WayPoint_OS(kk,:) = WayPoint_temp1;
                    case 2
                        WayPoint_OS(kk,:) = WP_3Ship(WayPoint_temp1(1,:),Dis_temp(1),WayPoint_temp1(2,:),Dis_temp(2));
                    case 3
                        WayPoint_OS(kk,:) = WP_4Ship(Risk_value',WayPoint_temp1);
                end
                kk=kk+1;
            end
            [m_wp,n_wp] = size(WayPoint_OS);
            t_wp=t*ones(m_wp,1);
            Boat(OS).WP_data = [Boat(OS).WP_data;t_wp,WayPoint_OS]; %�����͵ó�����tʱ�����г������ۺ�waypoint
            Boat(OS).WayPoint = WayPoint_OS;
            
            TCPA_OS=sum(Risk_temp(:,3));
            DCPA_OS=sum(Risk_temp(:,4));
            Dis_OS=sum(Risk_temp(:,5));
            Risk_OS=Risk_temp;
            
            Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
            Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
            Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
            %�����Ŀ�����ҵ���������ֵ�����õķ������ۺϷ���TCPA����ȣ�Ȼ��Dis��Ȼ����DCPA��
            %��100��10��1��Ϊϵ�����ֿ������ǲ�֪��Ч����Σ���Ҫ��һ���ĵ���
            Risk_OS(:,2)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
            Boat(OS).Risk=Risk_OS;
            Boat(OS).WayPoint_temp = WayPoint; %�����͵ó�����tʱ�����г������ۺ�waypoint
            
            %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
            
            if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
                % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
                Boat(OS).CAL=CAL0(OS,:);
            elseif  shipLabel(OS,2)==1
                % ��Ҷ˹�Ʋ�
                
                Boat(OS).CAL=CAL0(OS,:); %��Ҷ˹�ƶ����ó��ģ����ǵ�ǰʱ�̵�Boat(i).CAL
            end
            WayPoint_temp=[];
            Risk_level=0;
            k=1;
            for TS=1:1:Boat_Num
                if TS~=OS
                    if Boat(OS).Risk(k,2)~=0
                        if Boat(OS).Risk(k,2)>Risk_level
                            Risk_level=Boat(OS).Risk(k,2);
                            if Boat(OS).CAL(TS)==0
                                WayPoint_temp=Boat(OS).WayPoint_temp(k).WP0;
                            elseif Boat(OS).CAL(TS)==1
                                WayPoint_temp=Boat(OS).WayPoint_temp(k).WP1;
                            end
                        end
                    end
                else
                    continue
                end
                k=k+1;
            end
            if isempty(WayPoint_temp)
                Boat(OS).WayPoint=Boat(OS).goal;  %û��·�����ʱ��ֱ�����յ���ΪĿ���
            else
                Boat(OS).WayPoint=WayPoint_temp;
            end
            Boat(OS).HisWP = [Boat(OS).HisWP;t,Boat(OS).WayPoint];
            
            %% ����·���滮������
            % �ڵ�ǰʱ�����¼���FM������������
            %   1.֮ǰ��FM�������Ѿ��þ�����Ҫ���¼��㣻
            %   2.�������µ�·���������һ��·��������2���������ϣ�
            %   3.û���µ�·���㣬·������ΪĿ���
            if size(Boat(OS).HisWP,1)>=2   %�����ǵ�һ��·����
                if  norm(Boat(OS).HisWP(end-1,2:3)-Boat(OS).WayPoint)>=2*Res
                    Calculate_lable=1;
                end
            elseif Boat(OS).FM_lable>=size(Boat(OS).FMPos,1)
                Calculate_lable=1;
            elseif isempty(Boat(OS).curWP_label)
                Calculate_lable=1;
            else
                Calculate_lable=0;
            end
            
            if Calculate_lable==1
                %% ���Ƶ�ǰ������SCR
                RiskMap=ones(m,n);
                SCR_temp=zeros(m,n);
                
                for TS=1:1:Boat_Num
                    if TS~=OS
                        Boat_x = Boat(TS).pos(end,1);
                        Boat_y = Boat(TS).pos(end,2);
                        Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(OS,1);
                        
                        SCR_temp = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
                        RiskMap=RiskMap+SCR_temp;
                    end
                end
                FM_map=1./RiskMap;
                
                %% FM�㷨������
                start_point(1,1)  = round((Boat(OS).pos(1,1)+MapSize(1)*1852)/Res);
                start_point(1,2)  = round((Boat(OS).pos(1,2)+MapSize(2)*1852)/Res);
                
                end_point =round((Boat(OS).WayPoint+MapSize(1)*1852)/Res);
                t_count21=toc;
                [Mtotal, paths] = FMM(FM_map, end_point', start_point');
                
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
