function [CAL,chang,foreSum,aftSum] = CALjudge( Boat_x,Boat_y,Boat_theta,alpha,count_map,CAL0,MapSize,Res)
% InferGuidanceRange ���ڼ��㱾����ǰ���е����֣�Բ�ĵ�λ���ڱ�����ǰλ��
% ��������The angle guidance path planning algorithms for unmanned surface vehicle formations by using the fast marching method
% ���㷽���͹��򳡵ļ�������
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX,BoatY��
IGR_Dis=zeros(m,n);
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY< tand(90-alpha/2).*BoatX);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX);

IGR_Dis(IGR_Dis>20)=20;
%����û����R�����ƣ�ȫ��ͼ��Ч����ʱ��ʣ��Ϊ0�ľ��Ǵ�ͷ������
[fore_row,fore_col]=find(IGR_Dis==0); 
foreSum=sum(count_map(sub2ind(size(count_map),fore_row,fore_col)));

% Thetaȡ����������һ��
IGR_Dis=zeros(m,n);
Boat_theta=Boat_theta+pi;
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY< tand(90-alpha/2).*BoatX);
IGR_Dis=IGR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY<-tand(90-alpha/2).*BoatX);

IGR_Dis(IGR_Dis>20)=20;
%����û����R�����ƣ�ȫ��ͼ��Ч����ʱ��ʣ��Ϊ0�ľ��Ǵ�β������
[aft_row,aft_col]=find(IGR_Dis==0); 
aftSum=sum(count_map(sub2ind(size(count_map),aft_row,aft_col)));

%���ܵ����������жϣ�����Ҫ�Ա����ߵı��������������0.5���ң���λ��ԭCAL0
%Pr_fore��Pr_aft�������ǳɶԳ��ֵģ���һ���Ϳ�����
Pr_fore=foreSum/(foreSum+aftSum);  

if  Pr_fore>0.6
    %��ʱTS������OS��ͷ�Ŀ����Դ�TS��OS��CAL��0��OS��TS��CALΪ1
    CAL=1;

elseif Pr_fore<0.4
    %��ʱTS������OS��β�Ŀ����Դ�TS��OS��CAL��1��OS��TS��CALΪ0
    CAL=0;
else
    CAL=CAL0;
end

if  CAL==CAL0
    chang=0;  %CAL�ı�ı�־Ϊ0
else
    chang=1;  %CAL�ı�ı�־Ϊ0
end

end

