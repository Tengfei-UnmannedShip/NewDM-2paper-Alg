function RiskFieldValue = DrawAPF( Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize,APF_factor )
%航标参数初始化
%     Boat_x=Boat.State(i,1);                  %第i个船x坐标
%     Boat_y=Boat.State(i,2);                  %第i个船y坐标
%     Boat_theta=-Boat.State(i,3)/180*pi;       %第i个船艏向角度
%     Boat_Speed=Boat.State(i,4);              %第i个船速度大小
%     Boat_length=Boat.State(i,5);             %第i个船长
%     Boat_width=Boat.State(i,6);              %第i个船宽

[X,Y]=meshgrid(-MapSize(1):1:MapSize(1),-MapSize(2):1:MapSize(2));
[m,n]=size(X);

Boat_eta=APF_factor(1);
Boat_alfa=APF_factor(2);
BoatRiskFieldPeakValue=APF_factor(3);%风险最大值可以根据需要随意设置
Boat_Speed_Factor=APF_factor(4);%速度方向势场衰减因子，取值越大在速度方向上影响越大

RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue，即每一点的势场值。此处先将其归零为一个与X相同大小的0矩阵

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

BoatSpeedFactor=Boat_Speed_Factor*Boat_Speed;

%计算空间中点到第i个船边沿距离Dis
Dis=zeros(m,n);
Dis=sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0)+sqrt((BoatY/(Boat_Speed+1)).^2+BoatX.^2).*(BoatY>0);

%计算第i个船风险场
BoatRiskField=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis)./(Boat_alfa*Dis));

BoatRiskField(BoatRiskField>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;

%这里每个点在不同船下的场之间暂采用简单加和
RiskFieldValue=RiskFieldValue+BoatRiskField;

end

