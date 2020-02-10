%% (3.7�汾)����·���㣬�������ۺ�·����
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.A*��APF��ϵĵ����棬A*��APF����Ϊ����
%   0.1.�ٽ��ڵ���ѡ��Ϊ������һ��ѡ��
% 1.(2.0�汾)��չ������Ϊ��λ�ĵ�ͼ��
%   (2.5�汾)��WangNing��ShipDomain����ԭAPF����չ������Ϊ��λ�ĵ�ͼ
% 2.(3.0�汾)���·����
%   (3.5�汾)A*�㷨ֻ����δ��5����10����
%       ����1:ֻ����10������¼�����е�ʱ��ͽ��������ȫ��������Աȣ�ȷ��ֻ����10����׼ȷ�Ժ�Ч�ʣ�
%       ����2:����·���㣬Ȼ���һ�ξ��߼���10����Ȼ���ڵ�һ��forѭ����ʼʱ����״̬��ÿ5��������������5���ӣ�
%            t=60*5��������һ��·���㣬���û��·�����ˣ������յ�ΪĿ�꣬�������Ĺ����У�����ٴγ���·���㣬
%            ������·����ΪĿ�ꣻ
%   (3.7�汾)���õ���·���㣬���ٲ����ۺ�·����
%       ����1:�����ҵ��з��յ�Ŀ�괬���ҵ����з��յ��Ǹ���Ȼ���ҵ�����Ӧ��·����
%       ����2:һ�Ҵ�һ�Ҵ���·��
% 3.(4.0�汾)��ӱ�Ҷ˹

clear
clc
close all
tic;%tic1
%% ��ʼ����
MapSize=[10,10];
GoalRange=MapSize-[1,1];
Res=100;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

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
for i=1:1:Boat_Num
    Boat(i).reach=1; %�����־������Ŀ���ʱΪ0��δ����ʱΪ1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    Boat(i).HisCOG=[Boat(i).COG_rad,Boat(i).COG_deg];
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    Boat(i).pos=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    Boat(i).HisPos=Boat(i).pos;
    Boat(i).goal=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    
    Boat(i).WayPoint_temp = [];
    Boat(i).WayPoint = [];
    Boat(i).HisWP = [];
    
    Boat(i).As_lable=0; %��ʼʱ��As_lableΪ0��ֱ����һ��ʱ�̼���A*
    Boat(i).AsPos=[];
    Boat(i).AsCourse=[];
    Boat(i).AsCourse_deg=[];
    
end

for t=1:60:tMax
    t_count11=t_count12;    %ʱ�����
    
    %% ÿ��ʱ�̵�״̬����
    for i=1:1:Boat_Num
        if Boat(i).As_lable~=0
            
            Current_row=Boat(i).As_lable;
            Boat(i).pos = Boat(i).AsPos(Current_row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).AsCourse_deg(Current_row,:);
            Boat(i).COG_rad = Boat(i).AsCourse(Current_row,:);
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            Boat(i).As_lable = Boat(i).As_lable+1;
            
        else
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
        
        if norm(Boat(i).pos-Boat(i).goal)<=Res %������ǰ������ͬһ�����������Ϊ
            disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����Ŀ���']);
            Boat(i).reach=0;
        end
    end
    if Boat(1).reach==0 && Boat(2).reach==0 && ...
            Boat(3).reach==0 && Boat(4).reach==0
        disp([num2str(t),'ʱ��','���д�����Ŀ��㣬�������']);
        break
    end
    
    %% ·�������
    % =========================================================================
    % ���룺��ǰ��ÿ�Ҵ���λ��
    % �����ÿһ�ұ������е�ÿһ��Ŀ�괬��·����WP��
    % �������裺
    % 1.����ÿһ�Ҵ����Ĵ�ͷ��β������waypoint(WP),��ŷֱ�Ϊ0(��ͷ��)��1(��β��);
    % 2.�����ҵ����з��յ�Ŀ�괬�����ҵ����з��յ��Ǹ���Ȼ���ҵ�����Ӧ��·����
    % =========================================================================
    
    for OS=1:1:Boat_Num
        k=1;
        WP_label=[];
        Dis_temp=[];
        WayPoint_temp0=[];
        for TS=1:1:Boat_Num
            if TS~=OS
                v_os = Boat(OS).speed(end,:);
                course_os = Boat(OS).COG_deg(end,:);
                pos_os = Boat(OS).pos(end,:);
                v_ts = Boat(TS).speed(end,:);
                course_ts = Boat(TS).COG_deg(end,:);
                pos_ts = Boat(TS).pos(end,:);
                TSlength = ShipSize(TS,1);
                d_thre = 1*1852;                % d_threΪ�ж���ײ���յķ�����ֵ
                changeLabel = 0;                % ·����ľ����Ƿ�ɱ�
                % =========================================================================
                % ���ж�����ײ���գ�����Ҫ����waypoint��
                % �жϱ�׼���Ƿ�����ײ����
                % ע�⣺���ܳ���ԭ�������б���Ӧ���Ѿ���ȥ������Ԥ��·������ڹ���Ĵ�ͷһ������
                % ��ʱ�Ʋ���������������ݲ���A*����������ݵ�ԭ���������һ�ԣ���Ҫ��������
                % =========================================================================
                
                WP_label(k) = TS;
                
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
                
                %���㵱ǰ��Ŀ�괬�����޷��ա�TCPA��DCPA��������ľ���
                Risk_temp(k,:) = [TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp];
                
                % ��ʼ����waypiont
                WayPoint_temp0 = WP_2ship1(v_ts,course_ts,pos_ts,TSlength);
                WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0�Ǵ�ͷ��
                WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1�Ǵ�β��
                k=k+1;
            end
        end
        
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
    end
    %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
    for OS=1:1:Boat_Num
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
    end
    %% ���Ƶ�ǰÿ�Ҵ���SCR
    for i=1:1:Boat_Num
        
        Boat_x = Boat(i).pos(end,1);
        Boat_y = Boat(i).pos(end,2);
        Boat_theta = -Boat(i).COG_rad(end,:); %�˴�Ϊ������
        Boat_Speed = Boat(i).SOG(end,:);
        Shiplength = ShipSize(i,1);
        
        Boat(i).SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,2);
    end
    
    %% A*�㷨������
    for i=1:1:Boat_Num
        t_count21=t_count22;    %ʱ�����
        
        % �ڵ�ǰʱ�����¼���A*������������
        %   1.֮ǰ��A*�������Ѿ��þ�����Ҫ���¼��㣻
        %   2.�������µ�·���������һ��·��������2���������ϣ�
        %   3.û���µ�·���㣬·������ΪĿ���
        if size(Boat(i).HisWP,1)>=2   %�����ǵ�һ��·����
            if  norm(Boat(i).HisWP(end-1,2:3)-Boat(i).WayPoint)>=2*Res
                Calculate_lable=1;
            end
        elseif Boat(i).As_lable>=size(Boat(i).AsPos,1)
            Calculate_lable=1;
        elseif isempty(Boat(i).curWP_label)
            Calculate_lable=1;
        else
            Calculate_lable=0;
        end
        
        if Calculate_lable==1
            RiskMap=zeros(m,n);
            for k=1:1:Boat_Num
                if k~=i
                    RiskMap=RiskMap+Boat(k).SCR;
                end
            end
            
            % �ڵ�ǰʱ��ѡ���յ���ΪĿ�������������
            %   1.��ǰ������·���㣬��������·���㼯��Ϊ��
            %   2.·������뱾����ǰλ�õľ��������С��2��
            %   3.·�����ڱ������棬����Ϊ�����Ѿ�����·������
            theta = vec_ang(Boat(i).pos(end,:),Boat(i).WayPoint,Boat(i).COG_deg);
            if  norm(Boat(i).WayPoint-Boat(i).pos(end,:))<=2*Res || theta>=90
                end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);
            else
                end_x = round((Boat(i).WayPoint(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).WayPoint(1,2)+MapSize(2)*1852)/Res);
            end
            start_x = round((Boat(i).pos(end,1)+MapSize(1)*1852)/Res);
            start_y = round((Boat(i).pos(end,2)+MapSize(2)*1852)/Res);
            start_theta = Boat(i).COG_rad(end,:);   %��ʼ�����򣬻�����
            %A*�㷨��Ŀ��㣬��ÿ��ʱ�̣����·�����Ӧ��������·���㣬����·����󣬲���������յ�Ŀ���
            
            ShipLong=2*round(ShipSize(i,1)/Res/2);     %�����ȳ���2ȡ���ٳ�2��Ϊ�˳������õ�ShipLong/2ʱ,Ҳ���Ա�֤Ϊ����
            Movelength=round((Boat(i).speed(end,:)*60)/Res);  %����,ÿ�����н�����
            SurroundPointsNum=20; %������������n���A*
            valueAPF=2;  %APF�Ƴ��ļ�ֵ����
            NodeOpti=0;
            step_num=1000;
            map=RiskMap;
            
            [posData,courseData,courseData_deg] = Astar_step(step_num,map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize,Res);
            
            % ÿ�μ�����ֲ�A*֮�󣬶������Ϊ���µ�
            Boat(i).AsPos=posData;
            Boat(i).AsCourse=courseData;
            Boat(i).AsCourse_deg=courseData_deg;
            Boat(i).As_lable=1;     %ÿ�����¼��㣬����λ�õı�ʶλ���»ع�1���Ժ�ÿ�μ�1��֪�����굱ǰ�ļ���
            t_count22=toc;
            disp([num2str(i),'�Ŵ�����',num2str(step_num),'��������ʱ��: ',num2str(t_count22-t_count21)]);
        end
    end
    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵�����ʱ��: ',num2str(t_count12-t_count11)]);
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);
