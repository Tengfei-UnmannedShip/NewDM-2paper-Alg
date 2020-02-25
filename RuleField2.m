function RuleField = RuleField2( Boat_x,Boat_y,Boat_theta,Shiplength,Rule_eta,Rule_alfa,MapSize,Res,PeakValue,CAL)
% ��������ֻ�д�ͷ�ʹ�β��ָ������Ҫ�����ֵ�͵���Ӱ�췶Χ��
% RULEFIELD ����CAL�ж�OS���е�TS��Χ�Ĺ���RuleField
% ���巽������Ŀ�괬λ�ڱ�����fore sectionʱ����������������ײ����ʱ������Ϊ��·��(Give-way:1)��Ŀ�괬�Ա���Ϊֱ����(Stand-on:0).���򱾴�Ϊֱ������
% ������������µķ��ճ�������RuleField
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);
RuleDis=zeros(m,n);
RuleField=zeros(m,n);
% Rule_eta=3;
% Rule_alfa=0.1;
Rule_R=2.0*Shiplength;        %��Բ�뾶Ϊ2������
Rule_r=0.2*Shiplength;        %СԲ�뾶Ϊ0.5������
% R=2*1852;        %��Բ�뾶Ϊ2������
%�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX,BoatY��
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

if CAL==0  %��ʱ�����Ը�Ŀ�괬��0��������ΪGive-way��·��

    Rule_R=3*Shiplength;        %��Բ�뾶Ϊ2������
    Rule_r=0.2*Shiplength;        %СԲ�뾶Ϊ0.5������
    %��ʱ����Ӧ��Ŀ�괬��ͷ(fore section)������fore section�뾶ΪС��aft section�뾶Ϊ��
    %��ͷ�뾶С
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatX< tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>0 & BoatX>=tand(22.5)*BoatY);
    
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<=0 & -BoatX<=tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<=0 & -BoatX> tand(22.5)*BoatY);
    
elseif CAL==1   %��ʱ�����Ը�Ŀ�괬��1��������ΪStand-onֱ����
    
    Rule_R=1.5*Shiplength;        %��Բ�뾶Ϊ2������
    Rule_r=0.2*Shiplength;        %СԲ�뾶Ϊ0.5������
    
    %��ʱ����Ӧ��Ŀ�괬��β(aft section)������fore section�뾶Ϊ��aft section�뾶ΪС
    
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatX< tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX<0 & BoatX>=tand(22.5)*BoatY);
    
    RuleDis=RuleDis+(1/Rule_r)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>=0 & -BoatX<=tand(22.5)*BoatY);
    RuleDis=RuleDis+(1/Rule_R)*sqrt(BoatX.^2+BoatY.^2).*(BoatX>=0 & -BoatX> tand(22.5)*BoatY);
    
else     %��ʱ������
    RuleDis=zeros(m,n);
    
end

RuleField=PeakValue.*(exp(-Rule_eta*Rule_alfa*RuleDis)./(Rule_alfa*RuleDis));
RuleField(RuleField>PeakValue)=PeakValue;

end

