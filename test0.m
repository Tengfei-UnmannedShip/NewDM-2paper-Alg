% RP_all_L1=ceil(rand(4,6)*10)
tic
clc
clear
RP_all_L1=rand(5,6)*1000;  %6��RP��6��theta
Theta_L0=rand(6,1)*1000;

PrX_eq11=[];
PrX_eq12=[];
PrX02_eq1=[];
alpha_infer=1;
t11=toc;
for k_theta=1:1:6
    PrX01_eq1=[];
    for k_rp=1:1:5
        PrX01=-alpha_infer*(RP_all_L1(k_rp,k_theta)-Theta_L0(k_theta));
        PrX01=exp(PrX01);
        PrX01_eq1=[PrX01_eq1;PrX01];   %ÿһ�����²�һ������������ǵ�ǰtheta�ĵ�k_rp���ɴ���PrX�ķ���
    end
    sumK=sum(PrX01_eq1);      %ref2��������һ����һ������K���õ�����PrX0_eq1�ĺͣ������sumK
    % �õ�һ�����������ǵ�k_theta��theta�����е��PrXֵ
    PrX1=PrX01_eq1/sumK;  %�ɴ��k_rp��k_theta�µĹ�ʽ��1����ֵ
    %�������һ��ʱ�̹�ʽ��2����
    PrX_eq11=[PrX_eq11,PrX1];    %ÿ������һ�У���һ���ǵ�k_theta��theta�����е��PrXֵ
end
t12=toc;
t1=t12-t11
t21=toc;
for k_theta=1:1:6

    PrX02=exp(-alpha_infer*(RP_all_L1(:,k_theta)-Theta_L0(k_theta)));
    sumK=sum(PrX02);      %ref2��������һ����һ������K���õ�����PrX0_eq1�ĺͣ������sumK
    % �õ�һ�����������ǵ�k_theta��theta�����е��PrXֵ
    PrX2=PrX02/sumK;  %�ɴ��k_rp��k_theta�µĹ�ʽ��1����ֵ
    %�������һ��ʱ�̹�ʽ��2����
    PrX_eq12=[PrX_eq12,PrX2];    %ÿ������һ�У���һ���ǵ�k_theta��theta�����е��PrXֵ
end
t22=toc;
t2=t22-t21