%% (3.5�汾)A*�㷨ֻ����δ��5����10��
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
% 3.(4.0�汾)��ӱ�Ҷ˹

clear
clc
close all
tic;%tic1
%% ��ʼ����
MapSize=[8,8];
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
            Boat(i).As_lable = Boat(i).As_lable+1;
            Current_row=Boat(i).As_lable;
            Boat(i).pos = Boat(i).AsPos(Current_row,:);
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).COG_deg = Boat(i).AsCourse_deg(Current_row,:);
            Boat(i).COG_rad = Boat(i).AsCourse(Current_row,:);
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
            
        else
            Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
            Boat(i).HisPos=[Boat(i).HisPos;Boat(i).pos];
            Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        end
    end
    
    %% ·�������
    % =========================================================================
    % ���룺��ǰ��ÿ�Ҵ���λ��
    % �����ÿһ�ұ������е�ÿһ��Ŀ�괬��·����WP��
    % �������裺
    % 1.����ÿһ�Ҵ����Ĵ�ͷ��β������waypoint(WP),��ŷֱ�Ϊ0(��ͷ��)��1(��β��);
    % 2.�����⴬Ϊ���ĽǶȣ���8������(000-111)����1000��1111������forѭ������ʮ��������ת����2���Ʊ�ŵķ�ʽ����8������
    % 3.8�������е�ÿһ��������3���㣬���ݹ�ʽ���ҵ���3������м��(����3����͵����Ҵ��ľ������Ԫ���η�����)������м����������·����WP
    % ע�⣺
    % 1.��˭�ĽǶȳ��������⣬�������е�Ŀ�괬�ͱ������е�Ŀ�괬���е�������
    % =========================================================================
    
    for OS=1:1:Boat_Num
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
                    Dis_temp(k) = norm(pos_os-pos_ts);
                    WayPoint_temp0 = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
                    WayPoint(k).WP0=WayPoint_temp0(1:2);   %WP0�Ǵ�ͷ��
                    WayPoint(k).WP1=WayPoint_temp0(3:4);   %WP1�Ǵ�β��
                    k=k+1;
                end
            end
        end
        WP_Num = length(WP_label);
        Boat(OS).curWP_label=WP_label;
        kk = 1;
        if ~isempty(Boat(OS).curWP_label)   %���Boat(OS).curWP_label��Ϊ�գ�����������ײ���յĴ�
            for scenario = 2^WP_Num:1:2^(WP_Num+1)-1   %��WP_Num�ҷ��մ�����2^WP_Num������������WP_Numλ
                %��ÿһ������
                CAL_temp = dec2bin(scenario);
                for ts_i=1:1:WP_Num               %���ճ�����2���Ʊ�����ȡ��ÿһ�Ҵ��ڵ�ǰ�����µ�·����
                    if     CAL_temp(ts_i+1)=='0'     %dec2bin������������ַ�����������
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP0;
                    elseif CAL_temp(ts_i+1)=='1'
                        WayPoint_temp1(ts_i,:) = WayPoint(ts_i).WP1;
                    end
                end
                switch WP_Num
                    case 1
                        WayPoint_OS(kk,:) = WayPoint_temp1;
                    case 2
                        WayPoint_OS(kk,:) = WP_3Ship(WayPoint_temp1(1,:),Dis_temp(1),WayPoint_temp1(2,:),Dis_temp(2));
                    case 3
                        WayPoint_OS(kk,:) = WP_4Ship(Dis_temp,WayPoint_temp1);
                end
                kk=kk+1;
            end
        end
        Boat(OS).WayPoint_temp = WayPoint_OS; %�����͵ó�����tʱ�����г������ۺ�waypoint
    end
    %% Ŀ�괬����·���㱴Ҷ˹Ԥ�⣬ȷ����ʵ��CAL
    for i=1:1:Boat_Num
        if shipLabel(i,2)==0    % inferLabbel:�Ƿ��Ʋ�:0.���Ʋ�,1.�Ʋ�;
            % ���Ʋ⣬�����ı�CAL�������������CAL��ÿ�Ҵ���֪���˴˵�CAL
            Boat(i).CAL=CAL0(i,:);
        elseif  shipLabel(i,2)==1
            % ��Ҷ˹�Ʋ�
            
            Boat(i).CAL=CAL0(i,:); %��Ҷ˹�ƶ����ó��ģ����ǵ�ǰʱ�̵�Boat(i).CAL
        end
        
        CAL_temp1=Boat(i).CAL;
        WP_label1=Boat(i).curWP_label;
        scenario=CAL_temp1(WP_label1);
        WP_Num=length(Boat(i).curWP_label);
        
        switch WP_Num
            case 1        %ֻ��1��Ŀ�괬ʱ
                lable=scenario(1)+1;
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
            case 2        %��2��Ŀ�괬ʱ
                lable=2*scenario(1)+scenario(2)+1;
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
            case 3        %��3��Ŀ�괬ʱ
                lable=4*scenario(1)+2*scenario(2)+scenario(3)+1;
                disp(['��ǰΪ',num2str(i),'�Ŵ��ĳ��� ',num2str(lable)]);
                Boat(i).WayPoint=Boat(i).WayPoint_temp(lable,:);
        end
        Boat(i).HisWP = [Boat(i).HisWP;Boat(i).WayPoint];
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
        %       �ڵ�ǰʱ�����¼���A*������������
        %          1.֮ǰ��A*�������Ѿ��þ�����Ҫ���¼��㣻
        %          2.�������µ�·���������һ��·��������2���������ϣ�
        %          3.û���µ�·���㣬·������ΪĿ���
        if Boat(i).As_lable>=size(Boat(i).AsPos,1)||norm(Boat(i).HisWP(end-1,:)-Boat(i).WayPoint)>=2*Res ...
                ||isempty(Boat(i).curWP_label)
            RiskMap=zeros(m,n);
            for k=1:1:Boat_Num
                if k~=i
                    RiskMap=RiskMap+Boat(k).SCR;
                end
            end
            if  ~isempty(Boat(i).curWP_label)
                end_x = round((Boat(i).WayPoint(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).WayPoint(1,2)+MapSize(2)*1852)/Res);
            else
                end_x = round((Boat(i).goal(1,1)+MapSize(1)*1852)/Res);
                end_y = round((Boat(i).goal(1,2)+MapSize(2)*1852)/Res);
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
            step_num=2000;
            map=RiskMap;
            posData = [start_x,start_y];
            while size(posData,1)<=3
                step_num=2*step_num;
                [posData,courseData,courseData_deg] = Astar_step(step_num,map,start_x,start_y,start_theta,end_x,end_y,ShipLong,Movelength,SurroundPointsNum,valueAPF,NodeOpti,MapSize,Res);
            end
            %% ÿ�μ�����ֲ�A*֮�󣬶������Ϊ���µ�
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
