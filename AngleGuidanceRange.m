function AGRmap = AngleGuidanceRange( Boat_x0,Boat_y0,Boat_theta,alpha,R,MapSize,Res,PeakValue)
% ANGLEGUIDANCERANGE ���ڼ��㱾����ǰ���е����֣�Բ�ĵ�λ���ڱ�����β100m
% ��������The angle guidance path planning algorithms for unmanned surface vehicle formations by using the fast marching method
% ���㷽���͹��򳡵ļ�������
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
AGR_Dis=zeros(m,n);
AGRmap=zeros(m,n);
% R=200;        %��Բ�뾶Ϊ2������
% r=50;        %��Բ�뾶Ϊ0.5������

Boat_x=Boat_x0-200*sind(Boat_theta);       %��βĿ���x���꣬��βa2��������
Boat_y=Boat_y0-200*cosd(Boat_theta);       %��βĿ���y���꣬��βa2��������

%�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX,BoatY��
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0 & sqrt(BoatX.^2+BoatY.^2)<R);
AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY<tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R);
AGR_Dis=AGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX & sqrt(BoatX.^2+BoatY.^2)<R);


AGR_Dis(AGR_Dis>20)=PeakValue;
AGRmap=AGR_Dis;

end

