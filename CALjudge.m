function [CAL,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL0,MapSize,Res)
% InferGuidanceRange 用于计算本船当前航行的遮罩，圆心点位置在本船当前位置
% 来自论文The angle guidance path planning algorithms for unmanned surface vehicle formations by using the fast marching method
% 计算方法和规则场的计算类似
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
IGR_Dis=zeros(m,n);
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY< tand(90-alpha/2).*BoatX);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX);

IGR_Dis(IGR_Dis>20)=20;
%由于没有了R的限制，全地图有效，此时还剩下为0的就是船头的扇形
[fore_row,fore_col]=find(IGR_Dis==0); 
foreSum=sum(count_map(sub2ind(size(count_map),fore_row,fore_col)));

% Theta取反方向，再算一遍
IGR_Dis=zeros(m,n);
Boat_theta=Boat_theta+pi;
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY< tand(90-alpha/2).*BoatX);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX);

IGR_Dis(IGR_Dis>20)=20;
%由于没有了R的限制，全地图有效，此时还剩下为0的就是船尾的扇形
[aft_row,aft_col]=find(IGR_Dis==0); 
aftSum=sum(count_map(sub2ind(size(count_map),aft_row,aft_col)));

%不能单纯以数量判断，还是要对比两者的比例，如果比例在0.5左右，则位置原CAL0
%Pr_fore与Pr_aft这两个是成对出现的，算一个就可以了
Pr_fore=foreSum/(foreSum+aftSum);  

if  Pr_fore>0.6
    %此时TS出现在OS船头的可能性大，TS对OS的CAL是0，OS对TS的CAL为1
    CAL=1;

elseif Pr_fore<0.4
    %此时TS出现在OS船尾的可能性大，TS对OS的CAL是1，OS对TS的CAL为0
    CAL=0;
else
    CAL=CAL0;
end

if  CAL==CAL0
    chang=0;  %CAL改变的标志为0
else
    chang=1;  %CAL改变的标志为0
end

end

