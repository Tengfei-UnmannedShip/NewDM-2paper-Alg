function RR_points = ReachableRange( Boat_x,Boat_y,Boat_theta,alpha,R,r,MapSize,Res)
% ReachableRange 用于计算本船下一个时刻的可达域
% 返回值为所有可达域中的点的坐标
% 计算方法和规则场的计算类似
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
RR_Dis=zeros(m,n);

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0 & sqrt(BoatX.^2+BoatY.^2)<R & sqrt(BoatX.^2+BoatY.^2)>r );
RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY< tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R & sqrt(BoatX.^2+BoatY.^2)>r);
RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R & sqrt(BoatX.^2+BoatY.^2)>r);

RR_Dis(RR_Dis>20)=100;
[row,col]=find(RR_Dis~=0);
RR_points=[row,col];

end

