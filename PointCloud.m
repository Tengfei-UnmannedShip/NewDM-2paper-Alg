function Pos_pre = PointCloud( core,n,En,He ) 
%% �������ɳ�������Χ��ĳһ��Ŀ��λ�õ�ķ���2ά��̬�ֲ��ĵ���
% core Ϊ����λ��
% nΪ���ɵĵ�����
% En=0.005;     %�������ƫ�룬һ�����ԣ���
% He=0.1;       %�������ƫ�룬�������ԣ�����
% n=1000;
x=zeros(1,n);
y=zeros(1,n);
Ex=core(1);     %x����λ��
Ey=core(2);     %y����λ��
for i=1:n    %����Χ����ÿ����
    Ennx=randn(1)*He+En;
    x(i)=randn(1)*Ennx+Ex;
    Enny=randn(1)*He+En;
    y(i)=randn(1)*Enny+Ey;
end
Pos_pre=[x;y];

end

