function theta =NavAng(detaPos)
%NAVANG ����һ���������������Ϊ��YΪ������˳ʱ��Ϊ���ĵĽǶȣ������������
x=detaPos(1);
y=detaPos(2);
theta0=atand(abs(x/y)); %�õ�������y���γɵ��Ǹ����
if x==0 && y>0  %y������
    theta=0;
elseif x==0 && y<0  %y�Ḻ��
    theta=180;
elseif y==0 && x>0 %x������
    theta=90;
elseif y==0 && x<0 %x�Ḻ��
    theta=270;
elseif x>0 && y>0
    theta=theta0;
elseif x>0 && y<0
    theta=180-theta0;
elseif x<0 && y<0
    theta=180+theta0;
elseif x<0 && y>0
    theta=360-theta0;
end

end

