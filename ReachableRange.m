function RR_points = ReachableRange( Boat_x,Boat_y,course,alpha,R,r,MapSize,Res)
% ReachableRange ���ڼ��㱾����һ��ʱ�̵Ŀɴ���
% ����ֵΪ���пɴ����еĵ������
% ���㷽���͹��򳡵ļ�������
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
RR_Dis=zeros(m,n);

%�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX,BoatY��
BoatX = (X-Boat_x)*cos(course)+(Y-Boat_y)*sin(course);
BoatY = (Y-Boat_y)*cos(course)-(X-Boat_x)*sin(course);

RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatY>0 & BoatY> tand(90-alpha/2)*BoatX & sqrt(BoatX.^2+BoatY.^2)<=R & sqrt(BoatX.^2+BoatY.^2)>=r);
RR_Dis=RR_Dis+sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatY>0 & BoatY>-tand(90-alpha/2)*BoatX & sqrt(BoatX.^2+BoatY.^2)<=R & sqrt(BoatX.^2+BoatY.^2)>=r);

RR_Dis(RR_Dis>1)=100;
[row,col]=find(RR_Dis~=0);
% ����ĺ������������ҵ�ķ�ʽ�ҵ��ģ���ʼ��Ϊ0
% % ������դ�񻯵ĵ�ͼ�У���ʼ��Ϊ1����Ҳ�������������е�դ�񻯺󶼼�1��ԭ��
% % �����Ҫ������Ҳ����1����Ȼ��Զ�Ҳ������ʵĵ�
RR_points=[row,col];

end

