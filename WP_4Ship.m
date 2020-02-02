function x=WP_4Ship(Dis,waypoint)
%% 4�Ҵ���OS��3��TS����·�������
% һ�ֵ��͵���ţ�ٷ�broyden��������Է�����Ľ�
% ����ο���MATLAB��ֵ������Ӧ��_��Ҷ־��
% ʵ�� 6.5 ��ţ�ٷ���Broyden ��������ȫ���248ҳ��
% ����:
% % λ����Ϣpos�����ڼ��㵽��·����ļ�Ȩƽ��ֵ: 
% % ����λ��
% OS_pos=pos(1,:);
% % ����Ŀ�괬λ��
% TS_pos=pos(2:end,:);
% x0 Ϊ������ֵ
pos0=[0,0];
% tol Ϊ������ޣ����ȱʡĬ��Ϊ10��-10�η�
tol=1e-5;
% data ������ż�����м����ݱ��ڼ��������������

H0 = Jacobi4WP(pos0,Dis,waypoint);
H0 = inv(H0);
x1=pos0-(H0*Fun_WP(pos0,Dis,waypoint))' ;

n=1;

%���ó�ʼ���ʹ֮���Խ���ѭ��
wucha=0.1;
%ѭ������
while (wucha>tol) && (n<200)
    
    wucha=norm(x1-pos0);
    dx=x1-pos0;
    y=Fun_WP(x1,Dis,waypoint)-Fun_WP(pos0,Dis,waypoint);
    fenzi=dx*H0*y; %�Ǳ���
    H1=H0+(dx'-H0*y)*dx*H0/fenzi;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %��һ���൱��Ҫ��������Ҳ�Ƚ�ǿ�������
    temp_x0=pos0;
    pos0=x1;
    x1=temp_x0-(H1*Fun_WP(temp_x0,Dis,waypoint))'; %x1 �ĸ���
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %���� H ����
    H=H1;
    n=n+1;
    %data ��������м�����
    data(:,n)=x1;
    
end
x=x1;
end

