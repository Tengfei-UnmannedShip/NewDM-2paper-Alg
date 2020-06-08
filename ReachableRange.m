function RR_points = ReachableRange( Boat_x,Boat_y,course,alpha,R,r,MapSize,Res)
% ReachableRange 用于计算本船下一个时刻的可达域
% 返回值为所有可达域中的点的坐标
% 计算方法和规则场的计算类似
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
RR_Dis=zeros(m,n);

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(course)+(Y-Boat_y)*sin(course);
BoatY = (Y-Boat_y)*cos(course)-(X-Boat_x)*sin(course);

RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY> tand(90-alpha/2)*BoatX & sqrt(BoatX.^2+BoatY.^2)<=R & sqrt(BoatX.^2+BoatY.^2)>=r);
RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY>-tand(90-alpha/2)*BoatX & sqrt(BoatX.^2+BoatY.^2)<=R & sqrt(BoatX.^2+BoatY.^2)>=r);

RR_Dis(RR_Dis>1)=100;
[row,col]=find(RR_Dis~=0);
% 这里的横纵坐标是用找点的方式找到的，起始点为0
% % 但是在栅格化的地图中，起始点为1，这也是主程序中所有点栅格化后都加1的原因
% % 因此需要在这里也加上1，不然永远找不到合适的点
RR_points=[row,col];

end

