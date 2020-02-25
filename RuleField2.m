function RuleField = RuleField2( Boat_x,Boat_y,Boat_theta,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,PeakValue,CAL)
% 用于生成只有船头和船尾的指引场，要求最高值低但是影响范围大
% RULEFIELD 根据CAL判断OS眼中的TS周围的规则场RuleField
% 具体方法，当目标船位于本船的fore section时，并且两船存在碰撞风险时，本船为让路船(Give-way:1)，目标船对本船为直航船(Stand-on:0).否则本船为直航船。
% 计算避碰规则下的风险场，规则场RuleField
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
RuleDis=zeros(m,n);
RuleField=zeros(m,n);
% Rule_eta=3;
% Rule_alfa=0.1;
Rule_R=2.0*Shiplength;        %大圆半径为2倍船长
Rule_r=0.2*Shiplength;        %小圆半径为0.5倍船长
% R=2*1852;        %大圆半径为2倍船长
%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

if CAL==0  %此时本船对该目标船是0，即本船为Give-way让路船

    Rule_R=3*Shiplength;        %大圆半径为2倍船长
    Rule_r=0.2*Shiplength;        %小圆半径为0.5倍船长
    %此时本船应从目标船船头(fore section)过，即fore section半径为小，aft section半径为大
    %船头半径小
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatX< tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatX>=tand(22.5)*BoatY);
    
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<=0 & -BoatX<=tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<=0 & -BoatX> tand(22.5)*BoatY);
    
elseif CAL==1   %此时本船对该目标船是1，即本船为Stand-on直航船
    
    Rule_R=1.5*Shiplength;        %大圆半径为2倍船长
    Rule_r=0.2*Shiplength;        %小圆半径为0.5倍船长
    
    %此时本船应从目标船船尾(aft section)过，即fore section半径为大，aft section半径为小
    
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatX< tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatX>=tand(22.5)*BoatY);
    
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>=0 & -BoatX<=tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>=0 & -BoatX> tand(22.5)*BoatY);
    
else     %此时，规则场
    RuleDis=zeros(m,n);
    
end

RuleField=PeakValue.*(exp(-Rule_eta*Rule_alfa*RuleDis)./(Rule_alfa*RuleDis));
RuleField(RuleField>PeakValue)=PeakValue;

end

