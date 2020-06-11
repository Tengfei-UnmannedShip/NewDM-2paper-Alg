%% V2.0 �������ؿ���·���㷨�õ�PRM����
%     ����˼·������˼·���䣬ֻ���������·��ʱʹ�û����˹������е�PRM�㷨
% (1.1�汾)���ƹ���ԭ���򣬼���PRM������
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
% close all

tic;%tic1
%% ��ʼ����
% =========================================================================
% ������������
% =========================================================================
% 1.compliance:�������������:0.������COLREGs,1.����COLREGs;
% 2.inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
shipLabel=[
    1 1
    1 0
    1 0
    1 0];
%1~2λ��(�м��λ�ã�������ʼλ��)��3����(��)��4��ʼ����deg������Ϊ0����5��������ʱ����6��ⷶΧ��range��nm��
%����������������
ShipInfo=[
    0.0, 0.0,  20,    0,    3,  6
    0.0, 0.0,  20,  230,    4,  6
    0.0, 0.0,  18,  300,    5,  6
    0.0, 0.0,  17,  135,    5,  6
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
end


%�������߲�������
for i=1:1:Boat_Num
    %��Ŀ���λ�ù�һ��դ��λ���ϣ�������Ȼ������
    goal0=Goal_point(Boat(i).pos(1,1),Boat(i).pos(1,2),Boat(i).COG_deg,GoalRange*1852); %Boat(i)�ĳ�ʼĿ��λ�ã���λΪ��
    Boat(i).goal(1,1) =round(goal0(1,1)/Res)*Res;
    Boat(i).goal(1,2) =round(goal0(1,2)/Res)*Res;
    Boat(i).End=[];
    Boat(i).FM_lable=0; %��ʼʱ��FM_lableΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).decision_delay=0; %��ʼʱ��decision_delayΪ0��ֱ����һ��ʱ�̼���FM
    Boat(i).Current_row=0;
    for ii=1:1:Boat_Num
        %infer_label�Ƕ�Ŀ�괬���Ʋ�������ۼƣ���ʽΪ[�ϴ��Ʋ��ʱ�̣�TS�ܵ��Ʋ����]
        Boat(i).InferData(ii).infer_label=[0,0];
    end
    Boat(i).Theta_his=[];
    %ÿ�Ҵ���CAL����
    Boat(i).CAL=CAL0(i,:);       % Boat(OS).CAL��һ����������ʾOS������TS��CAL
    Boat(i).CAL_infer=CAL0(i,:); % Boat(OS).CAL_infer��һ�����������ڴ洢�Ʋ��CAL
end

% ��ʽ���߿�ʼ
for t=1:1:6    %tMax*2
    t_count11=t_count12;    %ʱ�����
    Boat0=Boat;
    % ÿʱ��״̬���£��ڳ��������еĶ�ֻ�Ǳ�Ҫ����Ϣ
    Boat=StateUpdate(Boat0,Boat_Num,t,Res);
    clear Boat0
    reach_label=0; %ÿ��ʱ�̹���
    reach_label=reach_label+Boat(1).reach+Boat(2).reach+Boat(3).reach+Boat(4).reach;
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
    % �жϱ����Ƿ������û���յ�ʱΪ1������Ϊ0����ˣ�����3�Ҵ������յ��reach_label<=1
    % �����DCPA�жϣ��п�����ʱ��DCPA��ʾû�з��գ����Ǻ������ǻ�������ַ���
    if reach_label<=1
        disp([num2str(t),'ʱ��','���д���ɱ������������']);
        break
    end
    
    for OS=1:1:Boat_Num    %Boat_Num
        %TODO �����ж������ļ򻯣��о���ǰ��̫�鷳
        %�жϵ�ǰiʱ������OS���ľ���������,compliance==1����������,��δ����Ŀ���
        if decisioncycle(t,ShipInfo(OS,5))&& shipLabel(OS,1)~=0 ...
                && Boat(OS).reach==1
            
            if Boat(OS).decision_delay<=round(20/ShipInfo(OS,5)) &&  Boat(OS).decision_delay>0    %ÿ�ξ���֮�󱣳�20s
                %                     && Boat(OS).Current_row>= size(Boat(OS).path,1)-10         %��Ԥ���ߵ�pathֻʣ����50����ʱ��ǿ�����¼���
                disp([num2str(t),'ʱ��',num2str(OS),'����',num2str(Boat(OS).decision_delay),'���ڱ������־���']);
                Boat(OS).decision_delay=Boat(OS).decision_delay+1;
            else
                disp([num2str(t),'ʱ��',num2str(OS),'����ʼ����']);
                %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
                if shipLabel(OS,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
                    % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
                    Boat(OS).CAL_infer=CAL0(OS,:); % Boat(OS).CAL_infer��һ����������ʾOS������TS��CAL
                elseif  shipLabel(OS,2)==1
                    Boat(OS).infercount=Boat(OS).infercount+1;
                    disp([' ',num2str(OS),'����',num2str(Boat(OS).infercount),'���Ʋ�']);
                    for  TS=1:1:Boat_Num
                        v_os      = Boat(OS).speed;
                        course_os = Boat(OS).COG_deg;
                        pos_os    = Boat(OS).pos;
                        v_ts      = Boat(TS).speed;
                        course_ts = Boat(TS).COG_deg;
                        pos_ts    = Boat(TS).pos;
                        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                        if CPA_temp(5)<=1852 && CPA_temp(6)>1   %TCPA>1���ų���ǰλ��ΪCPA�����
                            CurrentRisk=1; %����ײ����Ϊ1
                        else
                            CurrentRisk=0;
                        end
                        if TS~=OS && CurrentRisk==1   %�����з��ղ��Ʋ⣬�����Ʋ�
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
                            %��ʱos���ǳ���ts��
                            v_os      = Boat(TS).speed;
                            course_os = Boat(TS).COG_deg;
                            pos_os    = Boat(TS).pos;
                            
                            for ts_infer=1:1:Boat_Num
                                v_ts      = Boat(ts_infer).speed;
                                course_ts = Boat(ts_infer).COG_deg;
                                pos_ts    = Boat(ts_infer).pos;
                                CPA_infer = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                                if CPA_infer(5)<=1852 && CPA_infer(6)>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
                                    CurrentRisk_TS=1; %����ײ����Ϊ1
                                else
                                    CurrentRisk_TS=0;
                                end
                                if ts_infer~=TS && CurrentRisk_TS==1 %TS�ӽ��£�ts_infer(��������OS)��TS��ȷ����ײ����
                                    % ����0.1. �ҵ�TS�ӽ������п��ܵ�·����
                                    % �趨��һ�Ҵ��п����з��ղ���ܣ���Ϊʧ�ش������ǲ�����û�з��ջ���ν�Ĺ�ܡ�
                                    %      һ��Ŀ�괬û�з��մ��ˣ�Ŀ���ֻ��������յ�
                                    WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,ShipSize(ts_infer,1),0);
                                    %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
                                    WP1 = WayPoint_temp(1,1:2);
                                    %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
                                    WP2 = WayPoint_temp(1,3:4);
                                    
                                    if  Boat(OS).InferData(TS).infer_label(end,2)==1
                                        % TS֮ǰû���Ʋ�������ǵ�һ�Σ���Ҫ��ʼ��PrTheta
                                        %ע��0.1.�� ��һ����TS��Ŀ��㣬���TS�����ߣ�Ҳ����ֱ��������һ��ȥ
                                        ThetaState=[t,TS,TS,Boat(TS).goal,1,Boat(OS).InferData(TS).infer_label(end,2)];
                                        %��һ���Ʋ⣬���ʹ��Ԥ���PrTheta
                                        if CAL0(TS,ts_infer)==0
                                            Pr1=0.8;   %��WP1���Ŀ����Դ�
                                            Pr2=0.2;
                                        elseif CAL0(TS,ts_infer)==1
                                            Pr1=0.2;
                                            Pr2=0.8;   %��WP2���Ŀ����Դ�
                                        end
                                        ThetaState  =[ThetaState;
                                            t,TS,ts_infer,WP1,Pr1,Boat(OS).InferData(TS).infer_label(end,2);
                                            t,TS,ts_infer,WP2,Pr2,Boat(OS).InferData(TS).infer_label(end,2)];
                                    else
                                        % �Ѿ�����Ԥ��ļ�¼�󣬲�����һ��ʱ�̵�PrTheta��ֻ�Ʋ��Ԥ��
                                        % ����ʷ��¼����ȡ���µ�ThetaList0������ֻ���ڵ�ǰ��TS����ײ���յ�ts_infer�ż��뵱ǰ��ThetaList0
                                        WP_num=1;
                                        for k_inf=1:1:size(Boat(OS).Theta_his,1)
                                            if Boat(OS).Theta_his(k_inf,2)==TS && ...
                                                    Boat(OS).Theta_his(k_inf,3)==ts_infer && ...
                                                    Boat(OS).Theta_his(k_inf,7)==Boat(OS).InferData(TS).infer_label(end-1,2)
                                                % ����ȡ��TS�����Ŀ�����Ʋ�ֵ
                                                ThetaState  =[ThetaState;
                                                    Boat(OS).Theta_his(k_inf,1:6),Boat(OS).InferData(TS).infer_label(end,2)];
                                                %��ȡ������ʷThetaList0�ٹ���Pr1��Pr2
                                                %��Ϊ��ThetaList0�������У�ֻҪ��WP����ôWP1һ������WP2֮ǰ�����ԣ���һ�鵽�ľ���WP1
                                                if WP_num==1
                                                    Pr1=ThetaState(k_inf,6);
                                                else
                                                    Pr2=ThetaState(k_inf,6);
                                                end
                                                WP_num=WP_num+1;
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
                                end
                            end
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
                Boat0=Boat;
                % �����˵�ֻ��OS������
                Boat = MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t);
                clear Boat0
            end
        end
    end
    
%     if t==1000
%         Boat1000=Boat;
%         t1000=t_count12;
%         save('data0306-0100','Boat1000','t1000');
%         disp('1000s�����ѱ���');
%         clear Boat1000
%     elseif t==1500
%         Boat1500=Boat;
%         t1500=t_count12;
%         save('data0306-0100','Boat1500','t1500','-append');
%         disp('1500s�����ѱ���');
%         clear Boat1500
%     elseif t==2500
%         Boat2500=Boat;
%         t2500=t_count12;
%         save('data0306-0100','Boat2500','t2500','-append');
%         disp('2500s�����ѱ���');
%         clear Boat2500
%     elseif t==3500
%         Boat3500=Boat;
%         t3500=t_count12;
%         save('data0306-0100','Boat3500','t3500','-append');
%         disp('2000s�����ѱ���');
%         clear Boat3500
%     end
    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵����д���������ʱ��: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);

% Boat_end=Boat;
% t_end=t_count12;
% save('data0606-0100','Boat_end','t_end','-append');
% disp('���������ѱ���');
% clear Boat_end
