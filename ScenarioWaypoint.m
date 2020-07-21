function [WayPoint_OS,allpoint, R] = ScenarioWaypoint( R,W)
%% ��ǰ������3��Ŀ�괬���ۺ�·�������
% ���룺
% ��ǰ�ķ��ղ�����R(Risk_value)
% ��ǰ������·���㣺W(WayPoint_temp)
% ��������յõ����ۺ�·����
% ����˼·��ʹ��slove�����ҳ����еĽ⣬Ȼ��ɸѡ������ʵ��Ǹ���

syms x y
eq1=(R(2)*R(2)-R(1)*R(1))*x^2+(2*R(1)*R(1)*W(2,1)-2*R(2)*R(2)*W(1,1))*x+R(2)*R(2)*W(1,1)*W(1,1)-R(1)*R(1)*W(2,1)*W(2,1)+...
        (R(2)*R(2)-R(1)*R(1))*y^2+(2*R(1)*R(1)*W(2,2)-2*R(2)*R(2)*W(1,2))*y+R(2)*R(2)*W(1,2)*W(1,2)-R(1)*R(1)*W(2,2)*W(2,2);
eq2=(R(3)*R(3)-R(1)*R(1))*x^2+(2*R(1)*R(1)*W(3,1)-2*R(3)*R(3)*W(1,1))*x+R(3)*R(3)*W(1,1)*W(1,1)-R(1)*R(1)*W(3,1)*W(3,1)+...
        (R(3)*R(3)-R(1)*R(1))*y^2+(2*R(1)*R(1)*W(3,2)-2*R(3)*R(3)*W(1,2))*y+R(3)*R(3)*W(1,2)*W(1,2)-R(1)*R(1)*W(3,2)*W(3,2);

[x,y]=solve(eq1,eq2);
xx=double(x);
yy=double(y);

if size(xx)==1
    xx=[xx,xx];
end
if size(yy)==1
    yy=[yy,yy];
end

%�ж�����������������Ǿ����������������ʱ����Ҫ�Ѿ��������С������һ��
while   ~isreal(xx) ||    ~isreal(yy)
    if R(1)==max(R)
        R(1)=R(1)-0.5;
    elseif R(2)==max(R)
        R(2)=R(2)-0.5;
    elseif R(3)==max(R)
        R(3)=R(3)-0.5;
    end
    
    if R(1)==min(R)
        R(1)=R(1)+0.5;
    elseif R(2)==min(R)
        R(2)=R(2)+0.5;
    elseif R(3)==min(R)
        R(3)=R(3)+0.5;
    end
    
    [x,y]=solve(eq1,eq2);
    xx=double(x);
    yy=double(y);
    
    if size(xx)==1
        xx=[xx,xx];
    end
    if size(yy)==1
        yy=[yy,yy];
    end
    if R(1)==R(2) || R(1)==R(3) || R(2)==R(3)
        break
    end
end

WP_temp(1,:)=[xx(1),yy(1)];
WP_temp(2,:)=[xx(1),yy(2)];
WP_temp(3,:)=[xx(2),yy(1)];
WP_temp(4,:)=[xx(2),yy(2)];

%Ŀǰѡ����Ǿ����е�(0,0)����ĵ�
d=sum(WP_temp.*WP_temp,2);

[p,~]=find(d==min(min(d)));

WayPoint_OS=WP_temp(p(1),:);
allpoint=WP_temp;
end

