function newPoint = WP_regiontest(WP0,MapSize,pos)
% ������·������жϵ�ǰ��·�����Ƿ��ڵ�ͼ��Χ�ڣ����ڵĻ�����·����Ϊԭ·�����뱾��λ��������߽�Ľ���
% �������������x���ˣ�y���ˣ�xy�����ˣ�
region=(MapSize-[0.25,0.25])*1852;
if abs(WP0(1))<= region(1) && abs(WP0(2))<= region(2)  %���ڷ�Χ��
    newPoint = WP0;   %·���㲻��
else %������Χ֮��ʵ���Ͼ����Ҵӵ�ǰλ�ó�������·������ڱ߽��ϵĵ㣬����ֱ������Goal_point����
    %�����ҵ���·�����뱾��λ����y��������н�
    WP_pos=WP0-pos;
    y=[0,1];
    ang0=acos(dot(WP_pos,y)/(norm(WP_pos)*norm(y)));
    if WP_pos(1)>0
        ang=ang0;
    else
        ang=-ang0;
    end
    %����Goal_point�㷨������µ�·����
    newPoint = Goal_point(pos(1),pos(2),ang,region);
    
end
end

