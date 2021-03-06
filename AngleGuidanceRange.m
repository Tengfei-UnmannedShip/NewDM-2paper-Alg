function AGRmap = AngleGuidanceRange( Boat_x,Boat_y,Boat_theta,alpha,R,MapSize,Res,PeakValue)
% ANGLEGUIDANCERANGE 用于计算本船当前航行的遮罩，圆心点位置在本船当前位置
% 来自论文The angle guidance path planning algorithms for unmanned surface vehicle formations by using the fast marching method
% 计算方法和规则场的计算类似
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
AGR_Dis=zeros(m,n);
AGRmap=zeros(m,n);

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0 & sqrt(BoatX.^2+BoatY.^2)<R);
AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY<tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R);
AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R);


AGR_Dis(AGR_Dis>20)=PeakValue;
AGRmap=AGR_Dis;

end

