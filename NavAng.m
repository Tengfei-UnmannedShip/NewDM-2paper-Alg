function theta =NavAng(detaPos)
% NAVANG 输入一个航向向量，输出为正Y为正方向，顺时针为正的的角度，用于做航向角
% 注意：有时候是因为函数里面设置的条件不满足，所以这个参数没有在函数中被赋值。
% 也就说这个参数没有赋初值，也没有在函数里面后面的语句中赋值。
% 因此，写函数的时候一定要保证无论在哪一个条件语句中，都要有相应的赋值语句，才能保证该参数一定有值返回。
x=detaPos(1);
y=detaPos(2);
theta0=atand(abs(x/y)); %得到的是与y轴形成的那个锐角
if x==0 && y>0  %y轴正向
    theta=0;
elseif x==0 && y<0  %y轴负向
    theta=180;
elseif y==0 && x>0 %x轴正向
    theta=90;
elseif y==0 && x<0 %x轴负向
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

