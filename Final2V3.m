%% ���Ʋ�ĳ���V2������4���Ʋ�İ���2
% �Ȳ�����׷Խ�����İ���4


clear
clc
% close all
datatime=datestr(now,'mmdd-HHMM');
datatitle=strcat('data',datatime);
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
% %����������������
% ShipInfo=[
%     0.0, 0.0,  20,    0,    3,  6
%     0.0, 0.0,  20,  230,    4,  6
%     0.0, 0.0,  18,  300,    5,  6
%     0.0, 0.0,  17,  135,    5,  6
%     ];
% ׷Խ��������
ShipInfo=[
    0.0,  0.0,  18,    0,  3,  6
    0.0,  0.0,  13,    0,  4,  6
    0.0,  0.0,  16,  300,  5,  6
    0.0,  0.0,  13,  135,  5,  6
    ];

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
% ������COLREGs��CAL
CAL1=[
    2 1 1 0
    0 2 1 0
    0 0 2 1
    1 1 0 2];
% ���ݳ�ʼ�������õı���������״̬����Ҫ���ȱ��õĴ���д����ʶ��̫�鷳�����˹���ע������
% AvoS(1,1)��ʾ1�Ŵ��ڳ���1�����ȱ��õĴ���AvoS(1,2)��ʾ1�Ŵ��ڳ���2�����ȱ��õĴ�
% д������˼·������1���ҵ�������ʱ��˳��ĵ�һ��Ŀ�괬������2���ҵ�����˳ʱ��˳��ĵ�һ��Ŀ�괬
AvoS=[3 4
    4 3
    2 1
    1 2];

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
Res=18.52;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

for i=1:1:Boat_Num
    Boat(i).reach=1; %�����־������Ŀ���ʱΪ0��δ����ʱΪ1
    Boat(i).SOG = ShipInfo(i,3);                 %speed over ground����һ��Ϊ�Ե��ٶȣ���λ��
    Boat(i).speed = ShipInfo(i,3)*1852/3600;     %��һ��Ϊ�Ե��ٶȣ���λ�ף���
    Boat(i).COG_deg = ShipInfo(i,4);             %course over ground����һ��Ϊ��ʼ����deg��������Y����Ϊ0��
    Boat(i).COG_rad = ShipInfo(i,4)/180*pi;      %course over ground����һ��Ϊ��ʼ����rad��������Y����Ϊ0��
    Boat(i).HisCOG=[];
    %���м�λ�õ��Ƶĳ�ʼλ�ã��˴�pos��λΪ��,���ÿ��ʱ������һ��
    pos0=[ShipInfo(i,1)-Boat(i).speed*sind(Boat(i).COG_deg)*1250, ShipInfo(i,2)-Boat(i).speed*cosd(Boat(i).COG_deg)*1250];
    %�ѳ�ʼλ�ù�һ��դ��λ���ϣ�������Ȼ������
    Boat(i).pos(1,1) = round(pos0(1,1)/Res)*Res;
    Boat(i).pos(1,2) = round(pos0(1,2)/Res)*Res;
    Boat(i).RiskHis = [];
    Boat(i).HisPos=[];
    %��һ�ξ��ߵ�������
    Boat(i).path = [];
    Boat(i).infercount=0;
    Boat(i).reachWP=0;
end

%�������߲�������
for i=1:1:Boat_Num
    %��Ŀ���λ�ù�һ��դ��λ���ϣ�������Ȼ������
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    Boat(i).goal_real=goal0;
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    Boat(i).waypoint=[];
    Boat(i).centre=[];
    Boat(i).WayPoint_temp1=[];
    Boat(i).WP1=[];
    Boat(i).WP2=[];
    Boat(i).End=[];
    Boat(i).drawmap=[];
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    % ÿ�Ҵ���decision_label�����жϵ�ǰ�Ƿ���ߣ����û���쳣�������ϴξ������ݼ������С�
    % ����˵������ÿ�ξ��ߣ����ñ�Ҷ˹�жϣ��ж�û���⣬��CAL����
    % Ȼ���ھ���ʱ���û�з��գ���CAL���䣬��ôdecision_label=0����������
    % ����һ�θ���״̬�ǣ����ǰ�����һ�εľ�������
    Boat(i).decision_label=0;
    Boat(i).decision_count=0;   %���߼���
    Boat(i).Dechis.data=[];
    Boat(i).Dechis.startpos=[];
    Boat(i).Dechis.endpos=[];
    Boat(i).Dechis.map=[];
    Boat(i).Dechis.Scenariomap=[];
    Boat(i).Dechis.path=[];
    Boat(i).reachWP=0;
    Boat(i).index1=0;
    Boat(i).index2 = 0;
    Boat(i).dis1 = 0;
    Boat(i).dis2 = 0;
    for ii=1:1:Boat_Num
        %infer_label�Ƕ�Ŀ�괬���Ʋ�������ۼƣ���ʽΪ[�ϴ��Ʋ��ʱ�̣�TS�ܵ��Ʋ����]
        Boat(i).InferData(ii).infer_label=[0,0];
    end
    Boat(i).Theta_his=[];
    %ÿ�Ҵ���CAL����
    Boat(i).CAL=CAL0(i,:);       % Boat(OS).CAL��һ����������ʾOS������TS��CAL
    Boat(i).CAL_infer=CAL0(i,:); % Boat(OS).CAL_infer��һ�����������ڴ洢�Ʋ��CAL
end

%% Ԥ���㣬���ݳ�ʼ��������ÿ�Ҵ�������waypoint����дwaypoint�����˲�ͬ�ı��������������Ժ󱣳ֲ��䣬ֱ��CAL�ı�
for  OS=1:1:Boat_Num
    v_os          = Boat(OS).speed;
    course_os = Boat(OS).COG_deg;
    pos_os      = Boat(OS).pos;
    for TS=1:1:Boat_Num
        if TS~=OS
            v_ts          = Boat(TS).speed;
            course_ts = Boat(TS).COG_deg;
            pos_ts      = Boat(TS).pos;
            % ��һ����ÿ�Ҵ�����waypoint����
            WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,ShipSize(TS,1),0);
            %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
            Boat(OS).WP1 = [Boat(OS).WP1; WayPoint_temp(1,1:2)];
            %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
            Boat(OS).WP2 = [Boat(OS).WP2; WayPoint_temp(1,3:4)];
        end
    end
    WP_Num=Boat_Num-1;
    WayPoint_OS=[];
    % ������������жϣ��ڵ�ǰ�׶Σ�ÿ�Ҵ�ֻ�������������ϱ�������ʧ������״̬��
    % 1.����������COLREGs����ʱ����Risk_value���ð뾶
    CAL_ts=CAL0(OS,:);
    WayPoint_temp1=[];
    WayPoint_label=[];
    k=1;
    for ts_i=1:1:Boat_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
        if ts_i~=OS
            %�˴���ͼ4.1-1Ϊ��׼���������У�����000����˼�ǣ����д����ǶԱ�����·�������ӶԷ���ͷ����CAL���Ǵ�����
            if CAL_ts(ts_i)==0
                WayPoint_temp1(k,:) = Boat(OS).WP1(k,:);  %��ͷ·����
            elseif CAL_ts(ts_i)==1
                WayPoint_temp1(k,:) = Boat(OS).WP2(k,:);  %��β·����
            end
            if ts_i==AvoS(OS,1)
                WayPoint_label(k)=1;
            else
                WayPoint_label(k)=0;
            end
            k=k+1;
        end
    end
    x1=WayPoint_temp1(1,1);    y1=WayPoint_temp1(1,2);
    x2=WayPoint_temp1(2,1);    y2=WayPoint_temp1(2,2);
    x3=WayPoint_temp1(3,1);    y3=WayPoint_temp1(3,2);
    %������㹲��,�Ͱѵ�һ������΢��(0,0)Ųһ��
    if abs((y3-y1)*(x2-x1)-(y2-y1)*(x3-x1))<=0.000001
        WayPoint_temp1(1,1)=WayPoint_temp1(1,1)+20;
        dis([num2str(OS),'��״̬1������·���㹲�ߣ�������']);
    end
    [WayPoint_OS,centre] =ScenarioWaypoint( WayPoint_temp1,WayPoint_label);
    Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS];
    Boat(OS).centre=[Boat(OS).centre;centre];
    Boat(OS).WayPoint_temp1=[Boat(OS).WayPoint_temp1;WayPoint_temp1];
    % 2.������������COLREGs����ʱ����Risk_value2���ð뾶
    CAL_ts=CAL1(OS,:);
    WayPoint_temp1=[];
    WayPoint_label=[];
    k=1;
    for ts_i=1:1:Boat_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
        if ts_i~=OS
            %�˴���ͼ4.1-1Ϊ��׼���������У�����000����˼�ǣ����д����ǶԱ�����·�������ӶԷ���ͷ����CAL���Ǵ�����
            if CAL_ts(ts_i)==0
                WayPoint_temp1(k,:) = Boat(OS).WP1(k,:);  %��ͷ·����
            elseif CAL_ts(ts_i)==1
                WayPoint_temp1(k,:) = Boat(OS).WP2(k,:);  %��β·����
            end
            if ts_i==AvoS(OS,2)
                WayPoint_label(k)=1;
            else
                WayPoint_label(k)=0;
            end
            k=k+1;
        end
    end
    x1=WayPoint_temp1(1,1);    y1=WayPoint_temp1(1,2);
    x2=WayPoint_temp1(2,1);    y2=WayPoint_temp1(2,2);
    x3=WayPoint_temp1(3,1);    y3=WayPoint_temp1(3,2);
    %������㹲��,�Ͱѵ�һ������΢��(0,0)Ųһ��
    if abs((y3-y1)*(x2-x1)-(y2-y1)*(x3-x1))<=0.000001
        WayPoint_temp1(1,1)=WayPoint_temp1(1,1)+20;
        dis([num2str(OS),'��״̬2������·���㹲�ߣ�������']);
    end
    [WayPoint_OS,centre]=ScenarioWaypoint(WayPoint_temp1,WayPoint_label);
    Boat(OS).waypoint=[Boat(OS).waypoint;WayPoint_OS];
    Boat(OS).centre=[Boat(OS).centre;centre];
    Boat(OS).WayPoint_temp1=[Boat(OS).WayPoint_temp1;WayPoint_temp1];
    % 3.ʧ�أ�ֱ������Ŀ���
    Boat(OS).waypoint=[Boat(OS).waypoint;Boat(OS).goal_real];
    %��ɶԵ�ǰOS����3��·������ռ�
end
%ʶ���ʼ�����������д������ϱ�������ʱ
for  OS=1:1:Boat_Num
    Boat(OS).currentWP=Boat(OS).waypoint(1,:);
end
% Boat 1 �����ر�������
Boat(1).currentWP=Boat(1).waypoint(2,:);

t_wp=toc;
risk_factor_count=0;
%% ��ʽ���߿�ʼ
for t=1:1:4000   %tMax*2
    t_count11=t_count12;    %ʱ�����
%     Boat0=Boat;
    % ÿʱ��״̬���£��ڳ��������еĶ�ֻ�Ǳ�Ҫ����Ϣ
%     Boat=StateUpdate(Boat0,Boat_Num,t,Res);
%     clear Boat0
for i=1:1:Boat_Num
    if Boat(i).FM_lable~=0 && Boat(i).reach==1 %���Ѿ���ʼ���ߵ�û�е��յ�
        %����FMM�������ص㣬��Щ��֮�䲽���ر𳤣������Ҫ�õ�Ѱ���ķ���
        Index1=Boat(i).index1;
        Index2=Boat(i).index2;
        Dis1=Boat(i).dis1;
        dis=Boat(i).speed;
        
        [newPos,theta,Index1, Index2, Dis1, Dis2] = GetNextPoint(Boat(i).path, Index1, Index2, Dis1, dis);  %������Ĳ���·�������
        
        Boat(i).pos=newPos;
        Boat(i).HisPos=[Boat(i).HisPos;newPos];
        Boat(i).COG_deg =90-theta;
        Boat(i).COG_rad = Boat(i).COG_deg/180*pi;
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        Boat(i).index1=Index1;
        Boat(i).index2=Index2;
        Boat(i).dis1=Dis1;
        
    elseif Boat(i).reach==0  %�Ѿ������յ�
        Boat(i).pos = Boat(i).pos;
        Boat(i).COG_deg = Boat(i).COG_deg;
        Boat(i).COG_rad = Boat(i).COG_rad;
        
    elseif Boat(i).FM_lable==0  %û�о��߹���״̬
        
        Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
        Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
    end
    
    if norm(Boat(i).pos-Boat(i).goal)<=2*Res %������ǰ������ͬһ�����������Ϊ��������Ŀ���
        disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����Ŀ���']);
        Boat(i).reach=0;
    end
    
end
    reach_label=0; %ÿ��ʱ�̹���
    reach_label=reach_label+Boat(1).reach+Boat(2).reach+Boat(3).reach+Boat(4).reach;
    % �жϱ����Ƿ������û���յ�ʱΪ1������Ϊ0����ˣ�����3�Ҵ������յ��reach_label<=1
    % �����DCPA�жϣ��п�����ʱ��DCPA��ʾû�з��գ����Ǻ������ǻ�������ַ���
    if reach_label ==0
        disp([num2str(t),'ʱ��','���д���ɱ������������']);
        break
    end
    
    for OS=1:1:Boat_Num    
        %�жϵ�ǰiʱ������OS���ľ���������,compliance==1����������,��δ����Ŀ���
        
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(OS).reach==1  %&& OS~=4        %Boat 4 ��ʧ�ش���������
            disp([num2str(t),'ʱ��',num2str(OS),'����ʼ����']);
            %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
            if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
                % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
                Boat(OS).CAL_infer=CAL0(OS,:); % Boat(OS).CAL_infer��һ����������ʾOS������TS��CAL
            elseif  shipLabel(OS,2)==1
                Boat(OS).infercount=Boat(OS).infercount+1;
                disp([' ',num2str(OS),'����',num2str(Boat(OS).infercount),'���Ʋ�']);
                for  TS=1:1:Boat_Num
                    if TS~=OS
                        disp(['  ����',num2str(TS),'���ӽ�']);
                        TSinferlabel=[t,Boat(OS).InferData(TS).infer_label(end,2)+1];
                        Boat(OS).InferData(TS).infer_label=[Boat(OS).InferData(TS).infer_label;TSinferlabel];
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % ��ʽ��ʼTS�ӽǵ��Ʋ⣬��ʱTS�ͳ���OS���е�OS��һ�д�TS����
                        % ���룺��ǰ���Թ۲⵽�ĸ���״̬��
                        % ������Ʋ��TS״̬
                        %      1.TS����������CAL??����0/1��CAL�����͸���ֵ����ʽ���£�
                        %        TS��CAL��ʷ����ϸ��¼ÿһ�ε�CAL��Ԥ����
                        %        CalHis_TS=[t,TS,ts_infer,0,Pr(CAL=0);
                        %                   t,TS,ts_infer,0,Pr(CAL=0) ];
                        %        TS��CAL�Ʋ�����ȡ��������ֵ��û����ײ���յĴ�ȡ��ʼֵ���룬���磺
                        %        CAL_TS=[0,1,1,0]
                        %      2.OS�Ʋ��TS�Ŀ���·������һ�ŵ�ͼPreMap_TS��ÿһ����TS���ܾ�����λ�õļ���
                        %      3.���գ���OS�ķ���ֵ��
                        %        Boat(OS).Infer(TS).InferHis=CalHis_TS%�ۻ�
                        %        Boat(OS).Infer(TS).CAL=[t,CAL_TS]%�ۻ�
                        %        Boat(OS).Infer(TS).PreMap=PreMap_TS%ÿ�θ���
                        % �Ʋⷽ������Ҷ˹�Ʋ⣨����ͨ�����ɺ�����
                        % ���岽�裺
                        %        ����1.��ʽ(1)���µ�ǰλ�÷ֲ�
                        %        ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
                        %        ����3.������ͼ��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
                        %        ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·��
                        %        ����5.ÿһ��·���ع鵽�㣬ÿһ��·�������
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %% ����0.�Ʋ�ǰ��׼������
                        % ������
                        % ����0.1. �ҵ�TS�ӽ������п��ܵ�·����
                        % ����0.2. ���Ƶ�ǰTS���еķ��ճ������򳡺�������
                        ScenarioMap=zeros(m,n);
                        RRT_map=zeros(m,n);
                        % ����0.1. �ҵ�TS�ӽ������п��ܵ�·����
                        % �趨��һ�Ҵ��п����з��ղ���ܣ���Ϊʧ�ش������ǲ�����û�з��ջ���ν�Ĺ�ܡ�
                        %            һ��Ŀ�괬û�з��մ��ˣ�Ŀ���ֻ��������յ�
                        ThetaState=[];
                        if  Boat(OS).InferData(TS).infer_label(end,2)==1
                            % TS֮ǰû���Ʋ�������ǵ�һ�Σ���Ҫ��ʼ��PrTheta
                            %ע��0.1.�� ��һ����TS��Ŀ��㣬���TS�����ߣ�Ҳ����ֱ��������һ��ȥ
                            % �������Ƿ��ϱ�������ĳ�����ͬΪ0.5
                            ThetaState=[t,OS,TS,Boat(TS).goal,0.5/8,Boat(OS).InferData(TS).infer_label(end,2)];
                            %��һ���Ʋ⣬���ʹ��Ԥ���PrTheta
                            for infer_point=1:8
                                WP=Boat(TS).waypoint(infer_point,:);
                                if  isequl(Boat(TS).currentWP,WP)
                                    Pr=0.5;      %���ϱ�������ĳ���Ԥ��Ϊ0.5
                                else
                                    Pr=0.5/8;
                                end
                                ThetaState  =[ThetaState;
                                    t,OS,TS,WP,Pr,Boat(OS).InferData(TS).infer_label(end,2)];
                            end
                        else
                            % �Ѿ�����Ԥ��ļ�¼�󣬲�����һ��ʱ�̵�PrTheta��ֻ�Ʋ��Ԥ��
                            % ����ʷ��¼����ȡ���µ�ThetaList0
                            for k_inf=1:1:size(Boat(OS).Theta_his,1)
                                %����Boat(OS).Theta_his�ҵ���һ���Ʋ�TS������
                                if Boat(OS).Theta_his(k_inf,3)==TS && ...
                                        Boat(OS).Theta_his(k_inf,7)==Boat(OS).InferData(TS).infer_label(end-1,2)
                                    % ����ȡ��TS�����Ŀ�����Ʋ�ֵ
                                    ThetaState  =[ThetaState;
                                        Boat(OS).Theta_his(k_inf,1:6),Boat(OS).InferData(TS).infer_label(end,2)];
                                end
                            end
                        end
                        % ����0.2. ���Ƶ�ǰTS���еķ��ճ������򳡺�������
                        % ������������µķ��ճ�������RuleField
                        Boat_theta = -Boat(ts_infer).COG_rad; %�˴�Ϊ������
                        Boat_Speed =  Boat(ts_infer).SOG;
                        Shiplength =  ShipSize(ts_infer,1);
                        % ���ճ�
                        SCR_temp = ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                        cro_angle= abs(Boat(TS).COG_deg-Boat(ts_infer).COG_deg);
                        RRT_map=RRT_map+SCR_temp;
                        Rule_eta=2;
                        Rule_alfa=0.1;
                        % ����
                        CAL_Field0=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,0);
                        CAL_Field1=RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,1);
                        % ����������״̬���ո��ʵĵ��ӣ����״̬Ϊ0.5/0.5�������ɵ�·�������вο�����
                        CAL_Field=Pr1*CAL_Field0+Pr2*CAL_Field1;
                        % �ܵĻ�����
                        ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
                        
                        RiskMap=1./(ScenarioMap+1);
                        %��ɶ�TS���п���theta���ռ�������0.1.1��ɣ�
                        Theta=ThetaState(:,4:5); %��˳����ȡ��ǰ���е�theta��
                        %Theta������ɢ��
                        Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res)+1;
                        Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res)+1;
                        disp(['    ��ɶ�',num2str(TS),'������',num2str(size(Theta,1)),'������theta���ռ�']);
                        
                        % ��ǰTS��������
                        Boat_x=Boat(TS).pos(1,1);
                        Boat_y=Boat(TS).pos(1,2);
                        Boat_theta=-Boat(TS).COG_rad; %�˴�Ϊ������
                        alpha=30;
                        R=500;
                        
                        AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                        [AG_row,AG_col]=find(AFMfield~=0);
                        AG_points=[AG_row,AG_col];
                        
                        AG_map0=ones(size(AFMfield));
                        [AG_map, ~] = FMM(AG_map0, AG_points');
                        FM_map=min(AG_map,RiskMap);
                        %                             FM_map=RiskMap;
                        % TS��FMM��ʼλ��
                        start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
                        start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
                        %���start_point��FM_map��ֵ<0.001����FM����ʼ������0�㣬�޷�����·����Ҫ��0.001
                        if FM_map(start_point(1,1),start_point(1,2))<0.001
                            FM_map=FM_map+0.01;
                        end
                        % FM_map��FM�㷨������������ֵΪ1����˴���1ʱ��Ҫǿ����Ϊ1
                        FM_map(FM_map>1)=1;
                        [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
                        % ע�⣺�ó���InferMap��L0_paths�Ķ��������
                        % 1.InferMap��FMM�����ó����Ե�ǰTSλ��Ϊ������һ��ʱ�̵�CAL����Ϊ������ʵĵ�ͼ
                        % 2.L0_paths�ǵ�ǰ������theta�ó��ĳ�����·��
                        
                        %% ����1.��ʽ(1)������һ��ʱ�̵�λ�÷ֲ�����Eq2�ļ���ʹ��
                        % �ڲ���0���ҵ��˵�ǰʱ����Ҫ��Theta���ڱ����ҵ����еĿɴ��RR_points;
                        % ��������һʱ�̣���֪���´�ʲôʱ���Ʋ⣬���û��һ���ҵ�ȫ���Ŀɴ�㡣
                        % ������汾�иĳɣ�ÿ�μ�����һ���Ʋ�֮�󵽵�ǰ�����еĿɴ�㡣����ɴ��ļ��Ͽ��ܶ�Ҳ������
                        disp('    ��ʼEqu1�ļ���');
                        t_eq11=toc;
                        % ��ʱ������ΪTS��һ����״̬,���ҵ���һ���Ʋ��ʱ�̣��ٴ�HisPos�м�������ʱ��λ��
                        if   Boat(OS).InferData(TS).infer_label(end-1,1)==0
                            % Ϊ0ָ��ʼ���ã�֮ǰû���Ʋ������ͽ���ȡ��һ����HisPos
                            t_last=t-1;
                        else
                            % ��Ϊ0��֮ǰ�Ʋ��������ȡ��һ���Ʋ�ʱ��HisPos
                            t_last=Boat(OS).InferData(TS).infer_label(end-1,1);
                        end
                        Boat_x0    = Boat(TS).HisPos(t_last,1);
                        Boat_y0    = Boat(TS).HisPos(t_last,2);
                        Boat_theta=-Boat(TS).HisCOG(t_last,1);
                        delta_t=t-t_last;
                        speed=Boat(TS).speed;
                        [PrRR_points,PrePrX_eq1] = BayesEqu1(Boat_x0,Boat_y0,Boat_theta,speed,delta_t,L0_paths,Theta,Theta_end,FM_map,MapSize,Res);
                        % Boat(OS).inferdata�д洢�Ķ������һ�������ݣ�����һ����
                        Boat(OS).InferData(TS).infer_points=PrRR_points;
                        Boat(OS).InferData(TS).PrX=PrePrX_eq1;
                        t_eq12=toc;
                        disp(['    ���Equ1�ļ��㣬���µ�ǰ��λ�÷ֲ�����ʱ',num2str(t_eq12-t_eq11)]);
                        %% ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
                        pos_current(1)= round((Boat(TS).pos(1)+MapSize(1)*1852)/Res)+1;
                        pos_current(2)= round((Boat(TS).pos(2)+MapSize(2)*1852)/Res)+1;
                        row_index=ismember(PrRR_points,pos_current,'rows');
                        row_current=find(row_index==1);
                        if isempty(row_current)
                            disp('���󣡵�ǰ�㲻����һ��Ԥ����У�������ͣ');
                        end
                        % �õ���ʽ��2���ұߵ�һ������Pr(Xi=xi)����һ��
                        PrXTheta=PrePrX_eq1(row_current,:);  % ��ǰ������һ��ʱ�̵�����theta��PrXֵ
                        % �õ���ʽ��2���ұߵڶ�������Pr(theta0)������һ���Ĺ�ʽ��2����ֵ
                        PrTheta0=ThetaState(:,6)/sum(ThetaState(:,6)); %PrTheta��һ������ֹ����Խ��ԽС�����
                        PrTheta0=PrTheta0';        %��һ��Ԥ�⣬��һ���ǳ�ʼ�����ÿһ��theta���������
                        PrTheta = PrTheta0.*PrXTheta;
                        % ��ʽ��2������PrTheta
                        PrTheta = PrTheta/sum(PrTheta(:));
                        
                        % ���µ�ǰʱ�̵�ThetaList0�����浽Boat(OS).Theta_his��
                        ThetaState(:,6)=PrTheta';
                        Boat(OS).Theta_his=[Boat(OS).Theta_his;ThetaState];
                        disp('    ���Equ2�ļ��㣬���µ�ǰ��ThetaԤ��ֲ�');
                        %% ����3.������ͼ��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
                        % ����3.1. ���ո��º��PrTheta������MC��������theta��ĵ���
                        t_eq31=toc;
                        PointCloud_N=200;
                        Theta_MC=zeros(PointCloud_N,2);
                        for i_MC=1:PointCloud_N    %�����ɵ�ÿһ����
                            % PP=[0.1 0.2 0.7];
                            n_Theta=length(PrTheta);
                            MCsub=zeros(1,n_Theta);
                            MCsub(1)=PrTheta(1);
                            for i=2:1:n_Theta
                                MCsub(i)=MCsub(i-1)+PrTheta(i);
                            end
                            Select=find(MCsub>=rand);
                            row_select=Select(1);
                            %�õ���MC����������̬�ֲ����ɵ�Theta_MC���У���ʱTheta_MC������ȻΪʵ������
                            Theta_MC(i_MC,1)=Theta(row_select,1);
                            Theta_MC(i_MC,2)=Theta(row_select,2);
                        end
                        t_eq32=toc;
                        disp(['    ���Equ3�ļ��㣬������ͼ�������ɶ�Ӧÿһ��Theta��Ŀ����ƣ���ʱ',num2str(t_eq32-t_eq31)]);
                        %% ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬����PRM
                        t_eq41=toc;
                        % ����4.0.���Ƶ�ǰ�ĵ�ͼ��������ΧΪ�ϰ���������ǿ��ߵ�����
                        count_map=zeros(m,n);
                        RRTstart(1)=round(abs(( Boat(TS).pos(1)+MapSize(1)*1852)/Res))+1;
                        RRTstart(2)=round(abs((-Boat(TS).pos(2)+MapSize(2)*1852)/Res))+1;
                        RRTbraeklable=0;
                        for n_count=1:1:size(Theta_MC,1)    %���MC֮���ÿһ������·��
                            % ����4.1.��ÿһ��Theta_MC����ǰλ��Ϊ��㣬Theta_MCΪ�յ㣬����PRM
                            RRTend(1)=round(abs(( Theta_MC(n_count,1)+MapSize(1)*1852)/Res))+1;
                            RRTend(2)=round(abs((-Theta_MC(n_count,2)+MapSize(2)*1852)/Res))+1;
                            PrFocus=0.2+0.8*rand(1);
                            [L0,braeklable] = RRTPlanner(RRT_map,RRTstart,RRTend,speed,PrFocus) ;
                            RRTbraeklable=RRTbraeklable+braeklable;
                            count_map0=zeros(m,n);
                            % ����ÿһ��L0�ķ��ջ���InL0
                            % 1)L0���������ȡ�����ڵ�դ�����꣬ɾȥ�ظ��ģ���ֹһ������������
                            L0_point0=floor(L0);    %L0���������ȡ�����ڵ�դ������
                            L0_point=unique(L0_point0,'rows','stable');  %ɾȥ�ظ��У�����ԭ˳��
                            row_count=L0_point(:,1);
                            col_count=L0_point(:,2);
                            % ����4.2. ÿ��·���鵽դ����ϣ�����������ͼ
                            count_map0(sub2ind(size(count_map0),row_count,col_count))=1;
                            % ���յõ���count_map����MC֮��ÿһ���������ֵ����������
                            count_map=count_map+count_map0;
                        end
                        Boat(OS).InferData(TS).infermap(t).map=count_map;
                        % ����4.3.��count_map�ٻع鵽OS�Ĵ�ͷ�ʹ�β���ж�CAL
                        % ��count_map����OS�ĺ�������
                        % λ�û���OS��
                        Boat_x    = Boat(OS).pos(1,1);
                        Boat_y    = Boat(OS).pos(1,2);
                        Boat_theta=-Boat(OS).COG_rad; %�˴�Ϊ������
                        CAL_last  = Boat(OS).CAL_infer(TS);  %�˴�CAL��һ��ֵ����ǰ�Ʋ��OS��TS��CAL
                        alpha=45;
                        [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
                        Boat(OS).CAL_infer(TS)=CAL_current;  %�˴�CAL��һ��ֵ����ǰ�Ʋ��OS��TS��CAL
                        t_eq42=toc;
                        disp(['    ���Equ4�ļ��㣬����',num2str(PointCloud_N),'��MC·������ʱ',num2str(t_eq42-t_eq41)]);
                        disp(['    ��ɶ�',num2str(TS),'�����Ʋ⣬��TS�µĵ�CALΪ',num2str(CAL_current),'��CAL0=��',num2str(CAL0(OS,TS))]);
                    end
                end
                figure
                infer_map=zeros(m,n);
                for ts_map=1:1:Boat_Num
                    if ts_map~=OS &&  ismember(t,Boat(OS).InferData(ts_map).infer_label)
                        infer_map=infer_map+Boat(OS).InferData(ts_map).infermap(t).map;
                    end
                end
                % ��ʾ������ͼ,���鵱ǰ��count_map
                ss=pcolor(Y,X,infer_map);  %ע������Y��X���෴��
                set(ss, 'LineStyle','none');
                colorpan=ColorPanSet(0);
                colormap(colorpan);%����ɫ��
                hold on
                for plotship=1:1:4
                    %WTF:���������ĳ�ʼλ��
                    ship_icon(Boat(plotship).HisPos(1,1),Boat(plotship).HisPos(1,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(1,2),plotship)
                    %WTF:���������Ľ���λ��
                    ship_icon(Boat(plotship).HisPos(end,1),Boat(plotship).HisPos(end,2),ShipSize(plotship,1),ShipSize(plotship,2),Boat(plotship).HisCOG(end,2),plotship)
                    %WTF:���������ĺ���ͼ
                    plot(Boat(plotship).HisPos(:,1),Boat(plotship).HisPos(:,2),'k.-');
                end
                hold on
                % ��ʾThetaλ��
                plot(Theta(:,1),Theta(:,2),'r*')
                axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
                set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
                set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
                set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
                set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
                grid on;
                xlabel('\it n miles', 'Fontname', 'Times New Roman');
                ylabel('\it n miles', 'Fontname', 'Times New Roman');
                box on;
            end
            
            %% ���뱾�����ߣ�����������Ŀ��·�����FM�㷨
            Boat(OS).CAL=Boat(OS).CAL_infer;
            %                 Boat0=Boat;
            % �����˵�ֻ��OS������
            %                 Boat = MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t);
            % function Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t)
            % ���ߺ�����������ǰ��FM�����ͼ���ƣ����յ�ͼ�������ͼ������ͼ��
            % ��ʱBoatֻ������Boat(OS)�����ݣ�����Ҳֻ��OS�������TS��
            % Boat=Boat0;
            %% ���������ĺ��г�
            % ���Ƶ�ǰ��������Ŀ�괬��SCR
            t_count31=toc;    %ʱ�����
            ScenarioMap=zeros(m,n);
            RiskLabel=[];
            Risk_temp=[];
            k=1;
            k_ship=1;
            
            if   Boat(OS).decision_count==0     %�������û�о��߹�����һ���������ڻ���Ҫ���ߵ�
                Boat(OS).decision_label=1;
                Boat(OS).decision_kind=1;
                disp(['  ',num2str(OS),'�Ŵ��ĵ�һ�ξ���']);
            elseif Boat(OS).index1>size(Boat(OS).path,1)-2      % ������һ�����ߵĵ�������ˣ�Ҳ��Ҫ���¾���
                Boat(OS).decision_label=1;
                Boat(OS).decision_kind=3;
                disp(['  ',num2str(OS),'�Ŵ����й滮·��������ɣ��ٴξ���']);
            else
                Boat(OS).decision_label=0;
            end
            
            for TS=1:1:Boat_Num
                if TS~=OS
                    v_os = Boat(OS).speed(end,:);
                    course_os = Boat(OS).COG_deg(end,:);
                    pos_os = Boat(OS).pos;
                    v_ts = Boat(TS).speed(end,:);
                    course_ts = Boat(TS).COG_deg(end,:);
                    pos_ts = Boat(TS).pos;
                    dis_now=norm(pos_ts-pos_os);
                    Dis0=2*1852;% ֻ�������ľ���С��2����ʱ���ſ�ʼ���Ƶ�ͼ
                    CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                    DCPA=CPA_temp(5);
                    % ��������ʹĿ�괬�Ѿ��ͱ������룬��FM·���滮��ʱ��Ҳ�����Ŀ�괬���ǽ�ȥ
                    if  dis_now<Dis0 && DCPA<1000
                        disp(['  ',num2str(OS),'�Ŵ���',num2str(TS),'�Ŵ�����Ϊ',num2str(dis_now),'��Ҫ����Ŀ�괬�Ƴ�']);
                        Boat(OS).decision_label=Boat(OS).decision_label+1;
                        Boat(OS).drawmap=[Boat(OS).drawmap;t  TS];
                        % Ϊ������·������FM��ͼ�еĻ��䣬��Ҫ��·���㸽���Ĵ������ճ���С
                        DisWP_temp=pos_os-Boat(OS).currentWP;
                        DisWP=norm(DisWP_temp);
                        risk_factor=1;
                        if  DisWP>2*Res   %δ�ﵽ·���㣬Ŀ���Ϊ�յ�
                            DisWP_temp_ts=pos_ts-Boat(OS).currentWP;
                            DisWP_ts=norm(DisWP_temp_ts);
                            if  DisWP_ts<=2*1852   %TS��WP�ľ���С��1nmʱ������TS���շ�Χ���ˣ���Ҫ��СTS��Ӱ��
                                risk_factor=0.001;
                                disp(['    Ŀ�괬',num2str(TS),'����Ŀ��·����',num2str(DisWP_ts),'�������ճ�']);
                                risk_factor_count=risk_factor_count+1;
                            end
                        end
                        Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
                        Boat_Speed = Boat(TS).SOG(end,:);
                        Shiplength = ShipSize(TS,1);
                        SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                        
                        %������������µķ��ճ�������RuleField
                        cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
                        % ���CAL��OS��TS��CAL��Ϊ0��1
                        CAL=Boat(OS).CAL(TS);
                        Rule_eta=2;
                        Rule_alfa=0.1;
                        CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
                        ScenarioMap=ScenarioMap+(SCR_temp+CAL_Field)*risk_factor;
                    end
                end
            end
            RiskMap=1./(ScenarioMap+1);
            
            %% Ŀ���ȷ��
            if Boat(OS).reachWP==0   % reachWP==0˵����û��·���㣬��ʱĿ���Ϊ·����
                %    �жϷ���������Ѿ����ﳡ��·���㣬���Ϊ���յ�Ŀ���
                DisWP_temp=Boat(OS).pos-Boat(OS).currentWP;
                DisWP=norm(DisWP_temp);
                if  DisWP<4*Res
                    end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
                    end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
                    disp('    �ѵ���·���㣬Ŀ���Ϊ�յ�');
                    Boat(OS).reachWP=1;%����·�����reachWP=1
                    Boat(OS).reachWP=t; %��¼����ʱ��
                    Boat(OS).decision_label=Boat(OS).decision_label+1; % �ڸ���Ŀ���ʱ��decision_label�仯һ��
                else
                    end_point(1,2) =round((Boat(OS).currentWP(1,1)+MapSize(1)*1852)/Res)+1;
                    end_point(1,1) =round((Boat(OS).currentWP(1,2)+MapSize(2)*1852)/Res)+1;
                    disp('    �����У�Ŀ���Ϊ����·����');
                    Boat(OS).End=[Boat(OS).End;t,Boat(OS).currentWP,end_point];
                end
            else        %����·�����Ŀ����Ϊ�յ㣬����reachWPֻ����һ�Σ��˺󲻻��ٱ�
                end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res)+1;
                end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res)+1;
                disp('    �Ѿ���·���㣬Ŀ���Ϊ�յ�');
                Boat(OS).End=[Boat(OS).End;t,Boat(OS).goal,end_point];
            end
            
            if Boat(OS).decision_label~=0 % Boat(OS).decision_label��Ϊ0ʱ�ž��ߣ����򲻾���
                %% ���Ƶ�ǰ�����ĺ�������
                Boat_x=Boat(OS).pos(1,1);
                Boat_y=Boat(OS).pos(1,2);
                Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
                % Shiplength = ShipSize(OS,1);
                alpha=60;    %30����2*18.52�ķֱ�����̫С�ˣ�������AG_map�Ͽ��ڴ�����һ������ļ�̣������ڿ��ڸ�����������������һ����
                R=500;
                AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
                [AG_row,AG_col]=find(AFMfield~=0);
                AG_points=[AG_row,AG_col];
                AG_map0=ones(size(AFMfield));
                [AG_map, ~] = FMM(AG_map0, AG_points');
                FM_map=min(AG_map,RiskMap);
                start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res)+1;
                start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res)+1;
                %���start_point��FM_map��ֵ<0.001����FM����ʼ������0�㣬�޷�����·����Ҫ��0.001
                if FM_map(start_point(1,1),start_point(1,2))<0.001
                    FM_map=FM_map+0.01;
                end
                % FM_map��FM�㷨������������ֵΪ1����˴���1ʱ��Ҫǿ����Ϊ1
                FM_map(FM_map>1)=1;
                t_count32=toc;
                disp(['  ',num2str(OS),'�Ŵ����㺽�г���ʱ: ',num2str(t_count32-t_count31)]);
                %% FM�㷨������
                
                %2. ·���滮
                %����ʼ����յ㵽�߽紦ʱ����Ϊ���ڲ���û�г��ֹ������������������ɾ��
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
                
                [FMoutput, paths] = FMM(FM_map,start_point',end_point');
                t_count22=toc;
                disp(['      ',num2str(OS),'�Ŵ�·���滮��ʱ: ',num2str(t_count22-t_count21)]);
                Boat(OS).FM_lable=Boat(OS).FM_lable+1;
                %3. ���ݴ���
                FMpath = paths{:};
%                 FMpath=fliplr(FMpath);
                path0=rot90(FMpath',2);
                PathData_temp = zeros(size(path0));
                PathData=zeros(size(path0));
                PathData_temp(:,1)=path0(:,1)*Res-MapSize(1)*1852;
                PathData_temp(:,2)=path0(:,2)*Res-MapSize(2)*1852;
                
                if Boat(OS).pos(1)~=PathData_temp(1,1) && Boat(OS).pos(2)~=PathData_temp(1,2)
                    %�����ǰλ�ò��Ǿ��ߵ���㣬��ѵ�ǰλ�÷ŵ����
                    PathData=[Boat(OS).pos;PathData_temp];
                end
                
                Boat(OS).path=PathData;     %���decision_label==0���� Boat(OS).path����ԭ״
                Boat(OS).index1 = 1;            %ÿ�����¾��ߣ���ǰ������1
                Boat(OS).index2 = 2;
                Boat(OS).dis1 = 0;
                Boat(OS).decision_count=Boat(OS).decision_count+1;
                iDec=Boat(OS).decision_count;
                Boat(OS).Dechis(iDec).data=[t,iDec,start_point,end_point];
                Boat(OS).Dechis(iDec).startpos=[Boat_x,Boat_y];   %������ʵλ�ã�����դ�����ݷ���ģ��Լ�С���
%                 Boat(OS).Dechis(iDec).map=AG_map;
%                 Boat(OS).Dechis(iDec).Scenariomap=ScenarioMap;
                Boat(OS).Dechis(iDec).path=PathData;
            end
        end
        
    end

    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵����д���������ʱ��: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);


t_end=t_count12;
save(datatitle,'Boat');
disp('���������ѱ���');

