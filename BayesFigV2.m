%% ��Ҷ˹��ͼ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���Ҵ���ʵʱ���У����Ҵ��ҵ����Ե�·�ߣ���һ��ʱ�����һ��
% 0.(1.0�汾)FM��APF��ϵĵ����棬FM��APF����Ϊ����
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

%% ��Ҷ˹�Ʋ�����˼·
% �Ʋ�Ĳ���
% ����1.��ʽ(1)���µ�ǰλ�÷ֲ�
% ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
% ����3.������ͼ��������ÿһ��·����ĵ��������ݵ�������Χ��ÿһ��·����ĵ���
% ����4.ÿһ�����Ƶ�λ����Ϊ�յ㣬�յ���������FM������n��·��
% ����5.ÿһ��·���ع鵽�㣬ÿһ��·�������

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

%% ���ݼ����봦��
%��ͼ����
% MapSize_temp=max(max(abs(Start_pos)))/1852;
load data0814-2107.mat

MapSize=[8,8];
GoalRange=MapSize-[0.75,0.75];
Res=18.52;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

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

ShipSize = [
    250, 30
    290, 45
    290, 45
    270, 40    ];
t0=100;
OS=1;
TS=2;
Boat_Num=4;

for pic=1:3
    tfor0=toc;
    t=pic*t0+t0;
    disp(['��һ��ʱ��',num2str(t),'����']);
    %% ����0��׼��������������һ����Ԥ��ֵ
    ScenarioMap=zeros(m,n);
    RRT_map=zeros(m,n);
    v_os      = Boat(TS).speed;
    course_os = Boat(TS).HisCOG(t,2);
    pos_os    = Boat(TS).HisPos(t,:);
    % ����0.1.2. TS�ӽ������п��ܵ�·����
    % �ɶԳ��֣���Ϊ·�����theta��
    % �趨��һ�Ҵ��п����з��ղ���ܣ���Ϊʧ�ش������ǲ�����û�з��ջ���ν�Ĺ�ܡ�
    %      һ��Ŀ�괬û�з��մ��ˣ�Ŀ���ֻ������ı���Ŀ��һ��
    WP1 = Boat(TS).waypoint(1,:);  % ��1���������ر�������ĵ�
    WP2 = Boat(TS).waypoint(2,:);  % ��2�����ǲ����ر�������ĵ�
    WP3 = Boat(TS).waypoint(3,:);  % ��3������Ŀ���
    if  size(Boat(OS).InferData(TS). infer_label,1)==1   %TS֮ǰû���Ʋ�������ǵ�һ��
        % ����0.1. �ҵ�����theta
        % �����ǳ�ʼ״̬ʱ����ֱ����ȡ�����ϸ�ʱ�̵�theta��ֻ�Ʋ��Ԥ�⣬û�б���
        % ����0.1.1. ��һ����TS��Ŀ��㣬���TS�����ߣ�Ҳ����ֱ��������һ��ȥ
        Boat(OS).InferData(TS). infer_label(1,1)=t;
        Boat(OS).InferData(TS). infer_label(1,2)=Boat(OS).InferData(TS). infer_label(1,2)+1;
        %��һ���Ʋ⣬���ʹ��Ԥ���PrTheta
        Pr1=0.8;   %��ʼ�׶Σ���Ϊ��WP1���������ر�������Ŀ����Դ�
        Pr2=0.1;
        Pr3=0.1;
        
        Boat(OS).InferData(TS).ThetaPr0=[Pr1,Pr2,Pr3];   %�����һ��ʱ�̵��Ʋ�Pr
        Boat(OS).InferData(TS).ThetaPr=[Pr1,Pr2,Pr3];   %������µ��Ʋ�Pr
        Boat(OS).InferData(TS).Theta_his  =[ t,Boat(OS).InferData(TS). infer_label(1,2),Pr1,Pr2,Pr3]; %����Ʋ���ʷ
    else    %�Ѿ�����Ԥ��ļ�¼�󣬲�����һ��ʱ�̵�PrTheta
        % ��Pr1,Pr2����һ��ʱ�̼̳е���ǰʱ��
        infer_label_new=Boat(OS).InferData(TS). infer_label(end,2)+1;
        Boat(OS).InferData(TS). infer_label=[Boat(OS).InferData(TS). infer_label;t,infer_label_new];
        Boat(OS).InferData(TS).ThetaPr0=Boat(OS).InferData(TS).ThetaPr;   %�����һ��ʱ�̵��Ʋ�Pr
        
    end
    %��ɶ�TS���п���theta���ռ�������0.1.1��ɣ�
    disp('��ɶ�TS���п���theta���ռ�');
    Theta=[WP1;WP2;WP3]; %TS���е�theta��
    Theta_end(:,1)=round((Theta(:,2)+MapSize(2)*1852)/Res);
    Theta_end(:,2)=round((Theta(:,1)+MapSize(1)*1852)/Res);
    
    for ts_infer=1:1:Boat_Num
        Pr1= Boat(OS).InferData(TS).ThetaPr0(1);
        Pr2= Boat(OS).InferData(TS).ThetaPr0(2);
        v_ts      = Boat(ts_infer).speed;
        course_ts = Boat(ts_infer).HisCOG(t,2);
        pos_ts    = Boat(ts_infer).HisPos(t,:);
        CPA_temp  = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA_temp=CPA_temp(5);
        TCPA_temp=CPA_temp(6);
        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
            CurrentRisk_TS=1; %����ײ����Ϊ1
        else
            CurrentRisk_TS=0;
        end
        if ts_infer~=TS && CurrentRisk_TS==1 %TS�ӽ��£�ts_infer(��������OS)��TS��ȷ����ײ����
            % ����0.2. ���Ƶ�ǰTS���еķ��ճ������򳡺�������
            % ������������µķ��ճ�������RuleField
            Boat_theta = -Boat(ts_infer).HisCOG(t,1); %�˴�Ϊ������
            Boat_Speed = Boat(ts_infer).SOG;
            % ���ճ�
            Shiplength = ShipSize(ts_infer,1);
            SCR_temp= ShipDomain(pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            cro_angle=abs(Boat(TS).HisCOG(t,2)-Boat(ts_infer).HisCOG(t,2));
            
            CAL=CAL0(TS,ts_infer);
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
    
    % ��ǰTS��������
    Boat_x=Boat(TS).HisPos(t,1);
    Boat_y=Boat(TS).HisPos(t,2);
    Boat_theta=-Boat(TS).HisCOG(t,1); %�˴�Ϊ������
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
    t_count01=toc;
    [InferMap, L0_paths] = FMM(FM_map,start_point',Theta_end');
    t_count02=toc;
    % ע�⣺�ó���InferMap��L0_paths�Ķ��������
    % 1.InferMap��FMM�����ó����Ե�ǰTSλ��Ϊ������һ��ʱ�̵�CAL����Ϊ������ʵĵ�ͼ
    % 2.L0_paths�ǵ�ǰ������theta�ó��ĳ�����·��
    disp(['   ���theta�ĳ���·��L0_paths���ռ�����ʱ',num2str(t_count02-t_count01)]);
    
    %% ����1.��ʽ(1)������һ��ʱ�̵�λ�÷ֲ�����Eq2�ļ���ʹ��
    % �ڲ���0���ҵ��˵�ǰʱ����Ҫ��Theta���ڱ����ҵ����еĿɴ��RR_points;
    % ��������һʱ�̣���֪���´�ʲôʱ���Ʋ⣬���û��һ���ҵ�ȫ���Ŀɴ�㡣
    % ������汾�иĳɣ�ÿ�μ�����һ���Ʋ�֮�󵽵�ǰ�����еĿɴ�㡣����ɴ��ļ��Ͽ��ܶ�Ҳ������
    disp('    ��ʼEqu1�ļ���');
    t_eq11=toc;
    % ��ʱ������ΪTS��һ����״̬,���ҵ���һ���Ʋ��ʱ�̣��ٴ�HisPos�м�������ʱ��λ��
    if  size(Boat(OS).InferData(TS). infer_label,1)==1
        % Ϊ1ָ��ʼ���ã�֮ǰû���Ʋ������ͽ���ȡ��һ����HisPos
        t_last=t-t0;
    else
        % ��Ϊ0��֮ǰ�Ʋ��������ȡ��һ���Ʋ�ʱ��HisPos
        t_last=Boat(OS).InferData(TS).infer_label(end-1,1);
    end
    Boat_x0    = Boat(TS).HisPos(t_last,1);
    Boat_y0    = Boat(TS).HisPos(t_last,2);
    Boat_theta=-Boat(TS).HisCOG(t_last,1);
    delta_t=t-t_last;
    speed=Boat(TS).speed;
    Boat_x=Boat_x0;
    Boat_y=Boat_y0;
    course=Boat_theta;
    
    % ��Ҷ˹�ƶϵĹ�ʽ1�����ݵ�ǰ��λ�ú�theta������һ����λ�÷ֲ�
    % ���룺Boat_x,Boat_y,Boat_theta,speed������TS����״̬
    %      Theta����0���ó���theta�������͸���
    %      MapSize,Res����ͼ��Ϣ
    % �����RR_points����һ��ʱ�����еĿɴ�㣬դ����ʽ����ͼ��ÿһ�������
    %      PrX_eq1��PrX��ֵ��ÿһ�д���һ���ɴ��RR_points��ÿһ�д���һ��theta
    % ����1.   ��ʽ(1)���µ�ǰλ�÷ֲ�
    %% ����1.1. ����L0��������ջ���
    % �ҵ�TS����㵽theta��·��L0
    Theta_L0=[];
    for k_theta=1:1:size(Theta,1)     %���ÿһ��theta
        %��L0���ݴ���,���ջ�������դ������ģ����еı�Ҷ˹�ƶ϶�����դ�������
        L0 = L0_paths{k_theta};
        L0 = rot90(L0',2);
        L0_integral = RiskIntegral(L0,FM_map);
        % �õ�ÿһ��theta��Ӧ�ĵ�ǰ��L0���ջ���ֵ
        Theta_L0=[Theta_L0,L0_integral];
    end
    t_res=Res/speed; %Ҫ��֤ÿ��������ǰ��1��
    if delta_t>t_res
        %����ϴ��Ʋ��ʱ�䵽��ǰʱ�̣�����ǰ������1����ʹ��delta_t
        t_step=delta_t+3;  %�����¼���5s�������㵽��һ����������
    else
        %����ϴ��Ʋ��ʱ�䵽��ǰʱ��̫����������ǰ��1����ʹ��t_res
        t_step=t_res+3;  %�����¼���5s�������㵽��һ����������
    end
    %% ����1.2. �ҳ�tʱ�����еĿɴ��Reachable points(Rb_points)
    % ��ɸѡAG_points�ķ���ȷ��ĳһ��ʱ�̵Ŀɴ�㼯�ϣ�r=V*(t-1),R=V*t��
    t_eq101=toc;
    % ��һ���ɴ���ǵ�ǰ�㣬�������Ʋ����������£����ܳ��������ƶ�����һ�������е����
    Rb_points(1,2)=ceil((Boat_x+MapSize(1)*1852)/Res);
    Rb_points(1,1)=ceil((Boat_y+MapSize(2)*1852)/Res);
    R_infer=speed*t_step;
    r_infer=0.8*R_infer;
    alpha=120;
    %RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
    RRpoint0=ReachableRange(Boat_x,Boat_y,course,alpha,R_infer,r_infer,MapSize,Res);
    %˳�����½����n*2�ľ�����˲����ڳߴ粻ͳһ�����⣬���õ�l*2�ľ���lΪ����3�����ڿɴ��ĸ���
    Rb_points=[Rb_points;RRpoint0];
    
    % ʹ��unique������ɾȥ��ͬ�ĵ㣨����ͬ���У�
    Rb_points=unique(Rb_points,'rows','stable');
    % unique(A,��stable��)ȥ�ظ�������Ĭ�ϵ�������unique(A,��sorted��)����sorted��һ��ʡ�Ե��ˡ�
    Rb_points=fliplr(Rb_points);
    Rb_points0=Rb_points;
    Rb_points=Rb_points0(2:end,:);
    while size(Rb_points,1)>70
        Rb_points=Rb_points(1:2:end,:);   %ԭ���Ŀɴ��Rb_points̫���ˣ�1.3�������㲻��ȥ��ɸѡ��300������
    end
    t_eq102=toc;
    disp(['      1.2�ҵ����пɴ�㣬�ɴ��ԭ��',num2str(size(Rb_points0,1)),'��������',num2str(size(Rb_points,1)),'��������ʱ',num2str(t_eq102-t_eq101)]);
    %% ����1.3. ����L1�����߻���
    % ��ÿһ��RR_point�ҵ���Ӧ��TS-Reachable-theta��·��L1
    t_eq131=toc;
    RP_all_L1=[];
    for k_rp=1:1:size(Rb_points,1)
        Reach_point_now=Rb_points(k_rp,:);
        %RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
        start_point(1,1) = Reach_point_now(1);
        start_point(1,2) = Reach_point_now(2);
        %TODO �����Ƿ���Ҫ���������
        [~, L1_paths] = FMM(FM_map,start_point',Theta_end');
        
        %�ظ�һ���L0�ļ��㣬ֻ���������Reach_point
        RP_L1=[];    %���ÿһ�������theta
        for k_theta=1:1:size(Theta,1)     %���ÿһ��theta
            %��L0���ݴ���,���ջ�������դ������ģ����еı�Ҷ˹�ƶ϶�����դ�������
            L1 = L1_paths{k_theta};
            L1=rot90(L1',2);
            %�õ����ǵ�ǰ����Ե�ǰTheta��L1����ֵ
            L1_integral=RiskIntegral(L1,FM_map);
            %�õ����ǵ�ǰ���������Theta��L1����ֵ��������
            RP_L1=[RP_L1,L1_integral];
        end
        %�õ�ÿһ�д���һ���ɴ��(RP,reach points)��ÿһ����һ��theta��L1
        % RP_all_L1=[RP1Theta1,RP1Theta2,RP1Theta3,RP1Theta4,RP1Theta5,RP1Theta6
        %            RP2Theta1,RP2Theta2,RP2Theta3,RP2Theta4,RP2Theta5,RP2Theta6
        %            RP3Theta1,RP3Theta2,RP3Theta3,RP3Theta4,RP3Theta5,RP3Theta6
        %            RP4Theta1,RP4Theta2,RP4Theta3,RP4Theta4,RP4Theta5,RP4Theta6
        %            RP5Theta1,RP5Theta2,RP5Theta3,RP5Theta4,RP5Theta5,RP5Theta6];
        RP_all_L1=[RP_all_L1;RP_L1];
    end
    t_eq132=toc;
    disp(['      1.3����L1�����߻�����ϣ�����ʱ',num2str(t_eq132-t_eq131)]);
    %% ����1.4. ���ÿһ��theta�����µĹ�ʽ��1��
    PrX_eq1=[];
    alpha_infer=1;
    for k_theta=1:1:size(Theta,1)
        PrX0=exp(-alpha_infer*(RP_all_L1(:,k_theta)-Theta_L0(k_theta)));
        sumK=sum(PrX0);      %ref2��������һ����һ������K���õ�����PrX0_eq1�ĺͣ������sumK
        % �õ�һ�����������ǵ�k_theta��theta�����е��PrXֵ
        PrX=PrX0/sumK;  %�ɴ��k_rp��k_theta�µĹ�ʽ��1����ֵ
        % �������һ��ʱ�̹�ʽ��2����
        % ÿ������һ�У���һ���ǵ�k_theta��theta�����е��PrXֵ
        % ���PrX_eq1��ÿһ�к�Ϊ1
        PrX_eq1=[PrX_eq1,PrX];
    end
    % Boat(OS).inferdata�д洢�Ķ������һ�������ݣ�����һ����
    PrRR_points=Rb_points;
    PrePrX_eq1=PrX_eq1;
    Boat(OS).InferData(TS).infer_points=PrRR_points;
    Boat(OS).InferData(TS).PrX=PrePrX_eq1;
    t_eq12=toc;
    disp(['    ���Equ1�ļ��㣬���µ�ǰ��λ�÷ֲ�����ʱ',num2str(t_eq12-t_eq11)]);
    
    %% ����2.��ʽ(2)���µ�ǰ����ͼ�ֲ�
    pos_current(1)= round((Boat(TS).HisPos(t,1)+MapSize(1)*1852)/Res)+1;
    pos_current(2)= round((Boat(TS).HisPos(t,2)+MapSize(2)*1852)/Res)+1;
    %     row_index=ismember(PrRR_points,pos_current,'rows');
    %     row_current=find(row_index==1);
    %     if isempty(row_current)
    %         disp('���󣡵�ǰ�㲻����һ��Ԥ����У�������ͣ');
    %         break
    %     end
    
    % ���ھ����˿ɴ�㣬��˺��п��ܲ��ڿɴ������ǻ��������Χ�У��ҵ�����ĵ㼴�ɡ�
    % ��������ĵ㣬����·���Ѿ�դ�񻯣���˰��������Ⱦ�����㣬��ͬ����ѡ��һ��
    dis_row= abs(PrRR_points(:,1)-pos_current(1));
    dis_col= abs(PrRR_points(:,2)-pos_current(2));
    dis_all=dis_row+dis_col;
    [value,row_current]=min(dis_all);
    disp(['    ��ʼEqu2�ļ��㣬��ǰTSλ���ڵ�',num2str(row_current),'���㣬����Ϊ',num2str(value)]);
    % �õ���ʽ��2���ұߵ�һ������Pr(Xi=xi)����һ��
    PrXTheta=PrePrX_eq1(row_current,:);  % ��ǰ������һ��ʱ�̵�����theta��PrXֵ
    % �õ���ʽ��2���ұߵڶ�������Pr(theta0)������һ���Ĺ�ʽ��2����ֵ
    PrTheta0=Boat(OS).InferData(TS).ThetaPr0/sum(Boat(OS).InferData(TS).ThetaPr0); %PrTheta��һ������ֹ����Խ��ԽС�����
           %��һ��Ԥ�⣬��һ���ǳ�ʼ�����ÿһ��theta���������
    PrTheta = PrTheta0.*PrXTheta;
    % ��ʽ��2������PrTheta
    PrTheta = PrTheta/sum(PrTheta(:));
    
    % ���µ�ǰʱ�̵�ThetaPr�����浽Boat(OS).Theta_his��
    Boat(OS).InferData(TS).ThetaPr=PrTheta;
    Boat(OS).InferData(TS).Theta_his  =[ t,Boat(OS).InferData(TS). infer_label(end,2),PrTheta]; %����Ʋ���ʷ
    
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
    RRTstart(1)=round(abs(( Boat(TS).HisPos(t,1)+MapSize(1)*1852)/Res))+1;
    RRTstart(2)=round(abs((-Boat(TS).HisPos(t,2)+MapSize(2)*1852)/Res))+1;
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
    Boat_x    = Boat(OS).HisPos(t,1);
    Boat_y    = Boat(OS).HisPos(t,2);
    Boat_theta=-Boat(OS).COG_rad; %�˴�Ϊ������
    CAL_last  = Boat(OS).CAL_infer(TS);  %�˴�CAL��һ��ֵ����ǰ�Ʋ��OS��TS��CAL
    alpha=45;
    [CAL_current,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL_last,MapSize,Res);
    Boat(OS).CAL_infer(TS)=CAL_current;  %�˴�CAL��һ��ֵ����ǰ�Ʋ��OS��TS��CAL
    t_eq42=toc;
    tfor1=toc;
    disp(['    ���Equ4�ļ��㣬����',num2str(PointCloud_N),'��MC·������ʱ',num2str(t_eq42-t_eq41)]);
    disp(['    ��ɱ��ֶ�',num2str(TS),'�����Ʋ⣬CALΪ',num2str(CAL_current),'(CAL0=',num2str(CAL0(OS,TS)),'),��ʱ',num2str(tfor1-tfor0)]);
    disp('===========================================================');
end