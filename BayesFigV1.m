%% ��Ҷ˹��ͼ
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
%   (5.3�汾)�µ�AFM���㷽����
%       �������������ۣ�ԭ����AFMȡֵ��û���õģ�ԭ�������ɷ�ʽ�����⣬���������
%          (1)AFM���Ͱ�ȫ������ȡ��Сֵ�����µĳ�
%          (2)FM���������뼰�����XY�������ҵ������෴����˵����˺ܶ�����·�����޸ĺ��ٴ����顣
%       ���岽�裺
%        1.�����þɵķ����ҵ�angle guidance���Ƴ������в���0�ĵ㣬�ҳ�����λ��
%        2.ȫ��1�ĵ�ͼ������ĵ�ļ��Ϸ���FMM�����������µĵ�ͼ
%        3.ע�⣬��angle guidance����ʱ���ϰ��������xy�����ģ�������FMM·���滮�����������յ�������·��xy���Ƿ���
%   (5.4�汾)�����AFM����������ڰ�ȫ��ͼ��0ֵ���޷����������
%   (5.5�汾)CAL�̶�ʱ��·���滮�����ճ���
%       ����1��������ݣ����¾���ʱת����դ��������ת���ˣ�ֱ�Ӵ�դ���������ҵ���Ӧ����Դ���Ϊ��㡣
%       ����2�����ϳɺ������Ѹ����ֶ����ϳɺ�����������ֻ�ṩʱ��
% 5.(6.0�汾)����CAL�Ʋ⣬���ؿ��巽�������εı�Ҷ˹���ƺͽ��Ԥ��
%     ����˼·����ʽ(1)���µ�ǰλ�÷ֲ�����ʽ(2)���µ�ǰ����ͼ�ֲ���������ͼ��������Χ��ÿһ��·����ĵ���
%             ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·����ÿһ��·���ع鵽�㣬
%             ͨ��������OS��ͷ��β�ĵ�Ķ�����ȷ��CAL
%     ����1.��ʽ(1)���µ�ǰλ�÷ֲ�
%     ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
%     ����3.������ͼ���ʣ���MC��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
%     ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·��
%     ����5.ÿһ��·���ع鵽�㣬ÿһ��·�������
%     ����6.ͨ��������OS��ͷ��β�ĵ�Ķ�����ȷ��CAL
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
    1 1
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
    Boat(i).HisPos=[0,Boat(i).pos];
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
    Boat(i).infer_label=zeros(1,Boat_Num);
    Boat(i).Theta_his=[];
end

for t=1:1:6    %tMax*2
    t_count11=t_count12;    %ʱ�����
    Boat0=Boat;
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
                    %% ��ʼ��Ҷ˹�Ʋ�--����ͨ�����ɺ���
                    % �Ʋ�Ĳ���
                    % ����1.��ʽ(1)���µ�ǰλ�÷ֲ�
                    % ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
                    % ����3.������ͼ��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
                    % ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·��
                    % ����5.ÿһ��·���ع鵽�㣬ÿһ��·�������
                    for  TS=1:1:Boat_Num
                        v_os      = Boat(OS).speed;
                        course_os = Boat(OS).COG_deg;
                        pos_os    = Boat(OS).pos;
                        v_ts      = Boat(TS).speed;
                        course_ts = Boat(TS).COG_deg;
                        pos_ts    = Boat(TS).pos;
                        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                        DCPA_temp=CPA_temp(5);TCPA_temp=CPA_temp(6);
                        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
                            CurrentRisk=1; %����ײ����Ϊ1
                        else
                            CurrentRisk=0;
                        end
                        if TS~=OS && CurrentRisk==1   %�����з��ղ��Ʋ⣬�����Ʋ�
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
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            
                            ScenarioMap=zeros(m,n);
                            v_os      = Boat(TS).speed;
                            course_os = Boat(TS).COG_deg;
                            pos_os    = Boat(TS).pos;
                            
                            for ts_infer=1:1:Boat_Num
                                
                                v_ts      = Boat(ts_infer).speed;
                                course_ts = Boat(ts_infer).COG_deg;
                                pos_ts    = Boat(ts_infer).pos;
                                CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
                                if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
                                    CurrentRisk_TS=1; %����ײ����Ϊ1
                                else
                                    CurrentRisk_TS=0;
                                end
                                if ts_infer~=TS && CurrentRisk_TS==1 %TS�ӽ��£�ts_infer(��������OS)��TS��ȷ����ײ����
                                    % ����0.1.2. TS�ӽ������п��ܵ�·����
                                    % �ɶԳ��֣���Ϊ·�����theta��
                                    % �趨��һ�Ҵ��п����з��ղ���ܣ���Ϊʧ�ش������ǲ�����û�з��ջ���ν�Ĺ�ܡ�
                                    %      һ��Ŀ�괬û�з��մ��ˣ�Ŀ���ֻ�������ı���Ŀ��һ��
                                    changeLabel = 0; %���ɱ�·����
                                    Shiplength = ShipSize(ts_infer,1);
                                    WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,Shiplength,changeLabel);
                                    %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
                                    WP1 = WayPoint_temp(1,1:2);
                                    %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
                                    WP2 = WayPoint_temp(1,3:4);
                                    if  Boat(OS).infer_label(TS)==0   %TS֮ǰû���Ʋ�������ǵ�һ��
                                        % ����0.1. �ҵ�����theta
                                        % �����ǳ�ʼ״̬ʱ����ֱ����ȡ�����ϸ�ʱ�̵�theta��ֻ�Ʋ��Ԥ�⣬û�б���
                                        % ����0.1.1. ��һ����TS��Ŀ��㣬���TS�����ߣ�Ҳ����ֱ��������һ��ȥ
                                        Boat(OS).infer_label(TS)=Boat(OS).infer_label(TS)+1;
                                        
                                        ThetaList0=[t,TS,TS,Boat(TS).goal,1,Boat(OS).infer_label(TS)];
                                        %��һ���Ʋ⣬���ʹ��Ԥ���PrTheta
                                        if CAL0(TS,ts_infer)==0
                                            Pr1=0.8;   %��WP1���Ŀ����Դ�
                                            Pr2=0.2;
                                        elseif CAL0(TS,ts_infer)==1
                                            Pr1=0.2;
                                            Pr2=0.8;   %��WP2���Ŀ����Դ�
                                        end
                                        
                                        ThetaList0  =[ThetaList0;
                                            t,TS,ts_infer,WP1,Pr1,Boat(OS).infer_label(TS);
                                            t,TS,ts_infer,WP2,Pr2,Boat(OS).infer_label(TS)];
                                        
                                    else    %�Ѿ�����Ԥ��ļ�¼�󣬲�����һ��ʱ�̵�PrTheta
                                        % ˼����ô��Pr1,Pr2����һ��ʱ�̼̳е���ǰʱ��
                                        ThetaList0=[t,TS,TS,Boat(TS).goal,1,Boat(OS).infer_label(TS)];
                                        % ֻ���ڵ�ǰ��TS����ײ���յ�ts_infer�ż���ThetaList0
                                        for k_inf=1:1:size(Boat(OS).Theta_his,1)
                                            if Boat(OS).Theta_his(k_inf,2)==TS && ...
                                                    Boat(OS).Theta_his(k_inf,3)==ts_infer && ...
                                                    Boat(OS).Theta_his(k_inf,7)==Boat(OS).infer_label(TS)
                                                
                                                ThetaList0  =[ThetaList0;
                                                    Boat(OS).Theta_his(k_inf,:)];
                                                
                                            end
                                        end
                                        Boat(OS).infer_label(TS)=Boat(OS).infer_label(TS)+1;
                                    end
                                    Theta=ThetaList0(:,4:5); %ֻ�е�ǰ���е�theta��
                                    Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res);
                                    Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res);
                                    % ����0.2. ���Ƶ�ǰTS���еķ��ճ������򳡺�������
                                    % ������������µķ��ճ�������RuleField
                                    Boat_theta = -Boat(ts_infer).COG_rad; %�˴�Ϊ������
                                    Boat_Speed = Boat(ts_infer).SOG;
                                    % ���ճ�
                                    SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
                                    cro_angle=abs(Boat(TS).COG_deg-Boat(ts_infer).COG_deg);
                                    CAL=CAL0(TS,ts_infer);
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
                            end          %��ɶ�TS���п���theta���ռ�������0.1.1��ɣ�
                            RiskMap=1./(ScenarioMap+1);
                            
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
                            start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res);
                            start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res);
                            step_length=[Res,Res];
                            while  FM_map(start_point(1),start_point(2))<0.001
                                start_temp = ang_point(Boat_x,Boat_y,Boat(OS).COG_deg,step_length);
                                start_point(1,2) = round((start_temp(1)+MapSize(1)*1852)/Res);
                                start_point(1,1) = round((start_temp(2)+MapSize(2)*1852)/Res);
                                step_length=step_length+step_length;
                            end
                            [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
                            % ע�⣺�ó���InferMap��L0_paths�Ķ��������
                            % 1.InferMap��FMM�����ó����Ե�ǰTSλ��Ϊ������һ��ʱ�̵�CAL����Ϊ������ʵĵ�ͼ
                            % 2.L0_paths�ǵ�ǰ������theta�ó��ĳ�����·��
                            %% ����1.   ��ʽ(1)���µ�ǰλ�÷ֲ������¸�ʱ����
                            
                            [RR_points,PrX_eq1] = BayesEqu1(Boat_x,Boat_y,L0_paths,Boat_theta,Boat(TS).speed,Theta,Theta_end,FM_map,MapSize,Res);
                            
                            %TODO ÿ�θ��¿ɴ��Ͷ�Ӧ��ֵ������Theta��һ��
                            Boat(OS).infer_points(TS)=RR_points;
                            Boat(OS).PrX(TS)=PrX_eq1;
                            %% ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
                            % �����������
                            PrTheta0=ThetaList0(:,6)/sum(ThetaList0(:,6)); %PrTheta��һ������ֹ����Խ��ԽС�����
                            PrTheta0=PrTheta0';        %��һ��Ԥ�⣬��һ���ǳ�ʼ�����ÿһ��theta���������
                            if  Boat(OS).infer_label(TS)==1   %TS֮ǰû���Ʋ�������ǵ�һ��
                                % ������Ҫ����if���
                                % ��һ��Ԥ�⣬��Ҫ�ҵ���ǰ������һ�����������
                                % ��ʱ������ΪTS��һ����״̬
                                Boat_x=Boat(TS).HisPos(end-1,1);
                                Boat_y=Boat(TS).HisPos(end-1,2);
                                Boat_theta=-Boat(TS).HisCOG(end-1,1);
                                [PrRR_points,PrePrX_eq1] = BayesEqu1(Boat_x,Boat_y,Boat_theta,Boat(TS).speed,Theta,MapSize,Res);
                            else
                                % ֮ǰ�������ֱ�Ӳ�����һ���Ľ��
                                PrRR_points=Boat(OS).infer_points(TS);
                                PrePrX_eq1=Boat(OS).PrX(TS);
                            end
                            pos_current(1)=round((Boat(TS).pos(1)+MapSize(1)*1852)/Res);
                            pos_current(2)=round((Boat(TS).pos(2)+MapSize(2)*1852)/Res);
                            row_index=ismember(PrRR_points,pos_current,'rows');
                            row_current=find(row_index==1);
                            if isempty(row_current)
                                disp('���󣡵�ǰ�㲻����һ��Ԥ�����');
                                break
                            end
                            % �õ���ʽ��2���ұߵ�һ������Pr(Xi=xi)����һ��
                            PrXTheta=PrePrX_eq1(row_current,:);  % ��ǰ������һ��ʱ�̵�����theta��PrXֵ
                            PrTheta = PrTheta0.*PrXTheta;
                            PrTheta = PrTheta/sum(PrTheta(:));  % ��ʽ��2������PrTheta
                            % ���µ�ǰʱ�̵�ThetaList0�����浽Boat(OS).Theta_his��
                            ThetaList0(:,6)=PrTheta';
                            Boat(OS).Theta_his=[Boat(OS).Theta_his;ThetaList0];
                            
                            %% ����3.������ͼ��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
                            % ����3.1. ���ո��º��PrTheta������MC��������theta�������
                            N=100;
                            En=0.005;     %�������ƫ�룬һ�����ԣ���
                            He=0.1;       %�������ƫ�룬�������ԣ�����
                            Theta_MC=zeros(N,2);
                            for i_MC=1:N    %�����ɵ�ÿһ����
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
                                Ennx=randn(1)*He+En;
                                Theta_MC(i_MC,1)=randn(1)*Ennx+Theta(row_select,1);
                                Enny=randn(1)*He+En;
                                Theta_MC(i_MC,2)=randn(1)*Enny+Theta(row_select,2);
                            end
                            
                            %% ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·��
                            MC_end(:,1)=round((Theta_MC(:,2)+MapSize(2)*1852)/Res);
                            MC_end(:,2)=round((Theta_MC(:,1)+MapSize(1)*1852)/Res);
                            [MCMap, MC_paths] = FMM(FM_map,start_point',MC_end');
                            % ����4.2. ÿ��·���鵽դ����ϣ�����������ͼ
                            count_map=zeros(m,n);
                            for n_count=1:1:size(MC_end,1)    %���MC֮���ÿһ������·��
                                count_map0=zeros(m,n);
                                L0=MC_paths{n_count};
                                L0=rot90(L0',2);
                                x0=L0(1:50,1); %�滮��·��ƽ������
                                y0=L0(1:50,2);
                                %ƽ������
                                y_new=smooth(y0);
                                x_new=smooth(x0);
                                x_new1=smooth(x_new);
                                y_new1=smooth(y_new);
                                
                                L0(1:50,1)=x_new1;
                                L0(1:50,2)=y_new1; %�ó����յ�ƽ�����L0
                                % ����ÿһ��L0�ķ��ջ���InL0
                                % 1)L0���������ȡ�����ڵ�դ�����꣬ɾȥ�ظ��ģ���ֹһ������������
                                L0_point0=floor(L0);    %L0���������ȡ�����ڵ�դ������
                                L0_point=unique(L0_point0,'rows','stable');  %ɾȥ�ظ��У�����ԭ˳��
                                row_count=L0_point(:,1);
                                col_count=L0_point(:,2);
                                count_map0(sub2ind(size(count_map0),row_count,col_count))=1;
                                % ���յõ���count_map����MC֮��ÿһ���������ֵ����������
                                count_map=count_map+count_map0;
                            end
                            % ����4.3.��count_map�ٻع鵽OS�Ĵ�ͷ�ʹ�β���ж�CAL
                            % ��count_map����OS�ĺ�������
                            % λ�û���OS��
                            Boat_x=Boat(OS).pos(1,1);
                            Boat_y=Boat(OS).pos(1,2);
                            Boat_theta=-Boat(OS).COG_rad; %�˴�Ϊ������
                            CAL_last=Boat(OS).CAL(TS);
                            alpha=45;
                            [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
                            Boat(OS).CAL(TS)=CAL_current;
                        end
                    end
                end
                Boat0=Boat;
                Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t);
                clear Boat0
            end
        end
    end
    
    if t==1000
        Boat1000=Boat;
        t1000=t_count12;
        save('data0306-0100','Boat1000','t1000');
        disp('1000s�����ѱ���');
        clear Boat1000
    elseif t==1500
        Boat1500=Boat;
        t1500=t_count12;
        save('data0306-0100','Boat1500','t1500','-append');
        disp('1500s�����ѱ���');
        clear Boat1500
    elseif t==2500
        Boat2500=Boat;
        t2500=t_count12;
        save('data0306-0100','Boat2500','t2500','-append');
        disp('2500s�����ѱ���');
        clear Boat2500
    elseif t==3500
        Boat3500=Boat;
        t3500=t_count12;
        save('data0306-0100','Boat3500','t3500','-append');
        disp('2000s�����ѱ���');
        clear Boat3500
    end
    t_count12=toc;    %ʱ�����
    disp([num2str(t),'ʱ�̵����д���������ʱ��: ',num2str(t_count12-t_count11)]);
    disp('===========================================================');
end
t3=toc;
disp(['����������ʱ��: ',num2str(t3)]);

Boat_end=Boat;
t_end=t_count12;
save('data0306-0100','Boat_end','t_end','-append');
disp('���������ѱ���');
clear Boat_end