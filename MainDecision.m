function Boat= MainDecision(Boat0,OS,Boat_Num,ShipSize,MapSize,Res,t)
%决策函数，包括当前的FM输入地图绘制（风险地图，规则地图，引导图）


Boat=Boat0;
[X,~]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
%% 建立本船的航行场
% 绘制当前本船眼中目标船的SCR
t_count31=toc;    %时间计数
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
%         d_thre = 1*1852;                % d_thre为判断碰撞风险的风险阈值
        
        CPA_temp=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,1500);
        DCPA_temp=CPA_temp(5);
        %如果DCPA_temp=0，后续会出现0所为被除数的事，因此最小的DCPA设置为1，如果太小的话，除完会很大，影响前面的参数
        if DCPA_temp<1
            DCPA_temp=1;
        end
        TCPA_temp=CPA_temp(6);
        if DCPA_temp<=1852 && TCPA_temp>1   %TCPA>1即排除啊当前位置为CPA的情况
            CurrentRisk=1; %有碰撞风险为1
        else
            CurrentRisk=0;
        end
        Dis_temp=norm(pos_os-pos_ts);
        risklevel=0;
        %计算当前的目标船、有无风险、TCPA、DCPA、两船间的距离
        Risk_temp= [Risk_temp;TS,CurrentRisk, TCPA_temp,DCPA_temp,Dis_temp,risklevel];
        
        k=k+1;
        if  CurrentRisk==1 %有碰撞风险为1
            
            Boat_theta = -Boat(TS).COG_rad(end,:); %此处为弧度制
            Boat_Speed = Boat(TS).SOG(end,:);
            Shiplength = ShipSize(TS,1);
            
            SCR_temp= ShipDomain( pos_ts(1),pos_ts(2),Boat_theta,Boat_Speed,Shiplength,MapSize,Res,200,2);
            
            %计算避碰规则下的风险场，规则场RuleField
            cro_angle=abs(Boat(OS).COG_deg-Boat(TS).COG_deg);
            disp(['本船（',num2str(Boat(OS).COG_deg),'）与',num2str(TS),'号船（',num2str(Boat(TS).COG_deg),'）夹角为',num2str(cro_angle)]);
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

% 绘制当前本船的航行遮罩
Boat_x=Boat(OS).pos(1,1);
Boat_y=Boat(OS).pos(1,2);
Boat_theta=-Boat(OS).COG_rad(end,:); %此处为弧度制
% Shiplength = ShipSize(OS,1);
alpha=30;    %30度在2*18.52的分辨率上太小了，在最后的AG_map上开口处会有一个诡异的尖刺，是由于开口附近的两个方格连在一起了
R=500;
AFMfield=AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,200);
[AG_row,AG_col]=find(AFMfield~=0);
AG_points=[AG_row,AG_col];
AG_map0=ones(size(AFMfield));
[AG_map, ~] = FMM(AG_map0, AG_points');
FM_map=min(AG_map,RiskMap);    %防止出现起始点或终点在0处，无法计算的情况

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
disp([num2str(OS),'号船计算航行场用时: ',num2str(t_count32-t_count31)]);
% 寻找指引点

TCPA_OS=sum(Risk_temp(:,3));
DCPA_OS=sum(Risk_temp(:,4));
Dis_OS=sum(Risk_temp(:,5));
Risk_OS=Risk_temp;

Risk_OS(:,3)=TCPA_OS./Risk_temp(:,3);
Risk_OS(:,4)=DCPA_OS./Risk_temp(:,4);
Risk_OS(:,5)=Dis_OS./Risk_temp(:,5);
% 这里的目的是找到风险最大的值，采用的方法是综合法，TCPA最紧迫，然后Dis，然后是DCPA，
% 用100、10、1作为系数区分开，但是不知道效果如何，需要进一步的调试
Risk_OS(:,6)=Risk_OS(:,2).*(100*Risk_OS(:,3)+10*Risk_OS(:,5)+Risk_OS(:,4));
% t_risk=t*ones(size(Risk_OS,1));
%% FM算法主程序
%路径点确定
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
if  Danger_TS==OS   %即当前没有风险船，目标点为终点
    
    end_point(1,2) =round((Boat(OS).goal(1,1)+MapSize(1)*1852)/Res);
    end_point(1,1) =round((Boat(OS).goal(1,2)+MapSize(2)*1852)/Res);
    disp('当前没有风险船，目标点为终点');
    
else     %最危险的船即为当前的目标位置路径点
    disp(['当前风险船有',num2str(Risk_count),'艘，最危险的船为',num2str(Danger_TS)]);
    if   size(Boat(OS).End,1)>=1 && Boat(OS).End(end,2)==Danger_TS   %不是第一次决策且危险船与上次相同
        end_point= Boat(OS).End(end,3:4);
    else
        
        course_ts = Boat(Danger_TS).COG_deg(end,:);
        pos_ts = Boat(Danger_TS).pos;
        TSlength = ShipSize(Danger_TS,1);
        changeLabel = 0; %不可变路径点
        
        WayPoint_temp =  WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel);
%       WayPoint_temp = WP_2ship(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,TSlength,changeLabel);
        if Boat(OS).CAL(Danger_TS)==0 %此时本船对该目标船是0，即本船为Stand-on直航船
            %此时本船应从目标船船头(fore section)过，即为船头的目标点
            WP= WayPoint_temp(1,1:2);
            disp('路径点为船头点');
        elseif Boat(OS).CAL(Danger_TS)==1  %此时本船对该目标船是1，即本船为Give-way让路船
            %此时本船应从目标船船尾(aft section)过，即为船尾的目标点
            WP= WayPoint_temp(1,3:4);
            disp('路径点为船尾点');
        end
        
        end_point(1,2) =round((WP(1,1)+MapSize(1)*1852)/Res);
        end_point(1,1) =round((WP(1,2)+MapSize(2)*1852)/Res);
        
    end
end

Boat(OS).End=[Boat(OS).End;t,Danger_TS,end_point];
%2.路径规划
if size(Boat(OS).End,1)>1 && norm(Boat(OS).End(end-1,3:4)-end_point)<=2 ... %不是第一次运算的话，需要判断是否要维持上次的决策
        && Boat(OS).Current_row <= size(Boat(OS).path,1)-10         %当预决策的path只剩不到30个点时，强制重新计算
    disp('目标位置变动不大，则不计算');
else
    %当起始点或终点到边界处时，认为在内部
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
    disp([num2str(OS),'号船路径规划用时: ',num2str(t_count22-t_count21)]);
    Boat(OS).FM_lable=Boat(OS).FM_lable+1;
    %3. 数据处理
    FMpath = paths{:};
    path0=rot90(FMpath',2);
    Boat(OS).AFMpath=path0;
    posData = zeros(size(path0));
    posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;
    posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;
    
    if Boat(OS).pos(1)~=posData(1,1) && Boat(OS).pos(2)~=posData(1,2)
        %如果当前位置不是决策的起点，则把当前位置放到起点
        posData=[Boat(OS).pos;posData];
    end
    
    %规划的路径平滑处理
    x0=posData(1:50,1);
    y0=posData(1:50,2);
    y_new=smooth(y0);
    x_new=smooth(x0);
    x_new1=smooth(x_new);
    y_new1=smooth(y_new);
    posData(1:50,1)=x_new1;
    posData(1:50,2)=y_new1;
    
    Boat(OS).path=posData;
    Boat(OS).Current_row=1;   %每次重新决策，当前行数置1
    Boat(OS).decision_count=1;
%     
%     figure
%     contourf(X,Y,Mtotal);  %带填充颜色的等高线图
%     hold on
%     plot(Boat(OS).HisPos(1,1),Boat(OS).HisPos(1,2),'ro');
%     hold on
%     plot(Boat(OS).goal(1),Boat(OS).goal(2),'r*');
%     hold on
%     plot(Boat(OS).path(:, 1), Boat(OS).path(:, 2), 'r-');
end
end

