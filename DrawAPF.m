function RiskFieldValue = DrawAPF( Boat_x,Boat_y,Boat_theta,Boat_Speed,MapSize,APF_factor )
%���������ʼ��
%     Boat_x=Boat.State(i,1);                  %��i����x����
%     Boat_y=Boat.State(i,2);                  %��i����y����
%     Boat_theta=-Boat.State(i,3)/180*pi;       %��i��������Ƕ�
%     Boat_Speed=Boat.State(i,4);              %��i�����ٶȴ�С
%     Boat_length=Boat.State(i,5);             %��i������
%     Boat_width=Boat.State(i,6);              %��i������

[X,Y]=meshgrid(-MapSize(1):1:MapSize(1),-MapSize(2):1:MapSize(2));
[m,n]=size(X);

Boat_eta=APF_factor(1);
Boat_alfa=APF_factor(2);
BoatRiskFieldPeakValue=APF_factor(3);%�������ֵ���Ը�����Ҫ��������
Boat_Speed_Factor=APF_factor(4);%�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��

RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue����ÿһ����Ƴ�ֵ���˴��Ƚ������Ϊһ����X��ͬ��С��0����

%�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX,BoatY��
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

BoatSpeedFactor=Boat_Speed_Factor*Boat_Speed;

%����ռ��е㵽��i�������ؾ���Dis
Dis=zeros(m,n);
Dis=sqrt(BoatX.^2+BoatY.^2).*(BoatY<=0)+sqrt((BoatY/(Boat_Speed+1)).^2+BoatX.^2).*(BoatY>0);

%�����i�������ճ�
BoatRiskField=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis)./(Boat_alfa*Dis));

BoatRiskField(BoatRiskField>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;

%����ÿ�����ڲ�ͬ���µĳ�֮���ݲ��ü򵥼Ӻ�
RiskFieldValue=RiskFieldValue+BoatRiskField;

end

