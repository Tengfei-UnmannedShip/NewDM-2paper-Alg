function Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t)
%���ߺ�����������ǰ��FM�����ͼ���ƣ����յ�ͼ�������ͼ������ͼ��


Boat=Boat0;
[X,~]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
%% ���������ĺ��г�
% ���Ƶ�ǰ��������Ŀ�괬��SCR
t_count31=toc;    %ʱ�����
ScenarioMap=zeros(m,n);
RiskLabel=[];
Risk_temp=[];
k=1;
for TS=1:1:Boat_Num
    if TS~=OS
        
        v_os = Boat(OS).speed(end,:);
        course_os = Boat(OS).COG_deg(end,:);
        pos_os = Boat(OS).pos;
        v_ts = Boat(TS).speed(end,:);
        course_ts = Boat(TS).COG_deg(end,:);
        pos_ts = Boat(TS).pos;
%         d_thre = 1*1852;                % d_threΪ�ж���ײ���յķ�����ֵ
        
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
        Risk_temp= [Risk_temp;TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp,risklevel];
        
        k=k+1;
        if  CurrentRisk==1 %����ײ����Ϊ1
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %�˴�Ϊ������
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            
            %������������µķ��ճ�������RuleField
            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
            disp(['������',num2str(Boat(OS).COG_deg),'����',num2str(TS),'�Ŵ���',num2str(Boat(TS).COG_deg),'���н�Ϊ',num2str(cro_angle)]);
            CAL=Boat(OS).CAL(TS);
            Rule_eta=2;
            Rule_alfa=0.1;
            CAL_Field= RuleField(pos_ts(1),pos_ts(2),Boat_theta,cro_angle,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,200,CAL);
            ScenarioMap=ScenarioMap+SCR_temp+CAL_Field;
        end
    else
        RiskLabel(TS)=3;
    end
end
RiskLabel=[1,RiskLabel];
Boat(OS).RiskHis=[Boat(OS).RiskHis;RiskLabel];
RiskMap=1./(ScenarioMap+1);

% ���Ƶ�ǰ�����ĺ�������
Boat_x=Boat(OS).pos(1,1);
Boat_y=Boat(OS).pos(1,2);
Boat_theta=-Boat(OS).COG_rad(end,:); %�˴�Ϊ������
% Shiplength = ShipSize(OS,1);
alpha=30;    %30����2*18.52�ķֱ�����̫С�ˣ�������AG_map�Ͽ��ڴ�����һ������ļ�̣������ڿ��ڸ�����������������һ����
R=500;
AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
[AG_row,AG_col]=find(AFMfield~=0);
AG_points=[AG_row,AG_col];
AG_map0=ones(size(AFMfield));
[AG_map, ~] = FMM(AG_map0, AG_points');
FM_map=min(AG_map,RiskMap);    %��ֹ������ʼ����յ���0�����޷���������

start_point(1,2) = round((Boat_x+MapSize(1)*1852)/Res);
start_point(1,1) = round((Boat_y+MapSize(2)*1852)/Res);
step_length=[18.52,18.52];
while  FM_map(start_point(1),start_point(2))<0.001
    start_temp = ang_point(Boat_x,Boat_y,Boat(OS).COG_deg,step_length);
    start_point(1,2) = round((start_temp(1)+MapSize(1)*1852)/Res);
    start_point(1,1) = round((start_temp(2)+MapSize(2)*1852)/Res);
    step_length=step_length+[18.52,18.52];
end

t_count32=toc;
disp([num2str(OS),'�Ŵ����㺽�г���ʱ: ',num2str(t_count32-t_count31)]);
% Ѱ��ָ����

TCPA_OS=sum(Risk_temp(:,3));
DCPA_OS=sum(Risk_temp(:,4));
Dis_OS=sum(Risk_temp(:,5));
Risk_OS=Risk_temp;

Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
% �����Ŀ�����ҵ���������ֵ�����õķ������ۺϷ���TCPA����ȣ�Ȼ��Dis��Ȼ����DCPA��
% ��100��10��1��Ϊϵ�����ֿ������ǲ�֪��Ч����Σ���Ҫ��һ���ĵ���
Risk_OS(:,6)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
% t_risk=t*ones(size(Risk_OS,1));
%% FM�㷨������
%·����ȷ��
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
if  Danger_TS==OS   %����ǰû�з��մ���Ŀ���Ϊ�յ�
    
    end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
    end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
    disp('��ǰû�з��մ���Ŀ���Ϊ�յ�');
    
else     %��Σ�յĴ���Ϊ��ǰ��Ŀ��λ��·����
    disp(['��ǰ���մ���',num2str(Risk_count),'�ң���Σ�յĴ�Ϊ',num2str(Danger_TS)]);
    if   size(Boat(OS).End,1)>=1 && Boat(OS).End(end,2)==Danger_TS   %���ǵ�һ�ξ�����Σ�մ����ϴ���ͬ
        end_point= Boat(OS).End(end,3:4);
    else
        
        course_ts = Boat(Danger_TS).COG_deg(end,:);
        pos_ts = Boat(Danger_TS).pos;
        TSlength = ShipSize(Danger_TS,1);
        changeLabel = 0; %���ɱ�·����
        
        WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel);
%       WayPoint_temp = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
        if Boat(OS).CAL(Danger_TS)==0 %��ʱ�����Ը�Ŀ�괬��0��������ΪStand-onֱ����
            %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������Ϊ��ͷ��Ŀ���
            WP= WayPoint_temp(1,1:2);
            disp('·����Ϊ��ͷ��');
        elseif Boat(OS).CAL(Danger_TS)==1  %��ʱ�����Ը�Ŀ�괬��1��������ΪGive-way��·��
            %��ʱ����Ӧ��Ŀ�괬��β(aft section)������Ϊ��β��Ŀ���
            WP= WayPoint_temp(1,3:4);
            disp('·����Ϊ��β��');
        end
        
        end_point(1,2) =round((WP(1,1)+MapSize(1)*1852)/Res);
        end_point(1,1) =round((WP(1,2)+MapSize(2)*1852)/Res);
        
    end
end

Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];
%2.·���滮
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
    
    [Mtotal, paths] = FMM(FM_map,start_point',end_point');
    t_count22=toc;
    disp([num2str(OS),'�Ŵ�·���滮��ʱ: ',num2str(t_count22-t_count21)]);
    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
    %3. ���ݴ���
    FMpath = paths{:};
    path0=rot90(FMpath',2);
    Boat(OS).AFMpath=path0;
    posData = zeros(size(path0));
    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
    if Boat(OS).pos(1)~=posData(1,1) && Boat(OS).pos(2)~=posData(1,2)
        %�����ǰλ�ò��Ǿ��ߵ���㣬��ѵ�ǰλ�÷ŵ����
        posData=[Boat(OS).pos;posData];
    end
    
    %�滮��·��ƽ������
    x0=posData(1:50,1);
    y0=posData(1:50,2);
    y_new=smooth(y0);
    x_new=smooth(x0);
    x_new1=smooth(x_new);
    y_new1=smooth(y_new);
    posData(1:50,1)=x_new1;
    posData(1:50,2)=y_new1;
    
    Boat(OS).path=posData;
    Boat(OS).Current_row=1;   %ÿ�����¾��ߣ���ǰ������1
    Boat(OS).decision_count=1;
%     
%     figure
%     contourf(X,Y,Mtotal);  %�������ɫ�ĵȸ���ͼ
%     hold on
%     plot(Boat(OS).HisPos(1,1),Boat(OS).HisPos(1,2),'ro');
%     hold on
%     plot(Boat(OS).goal(1),Boat(OS).goal(2),'r*');
%     hold on
%     plot(Boat(OS).path(:, 1), Boat(OS).path(:, 2), 'r-');
end
end

