% RP_all_L1=ceil(rand(4,6)*10)
tic
clc
clear
RP_all_L1=rand(5,6)*1000;  %6个RP，6个theta
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
        PrX01_eq1=[PrX01_eq1;PrX01];   %每一次往下补一个数，这个数是当前theta的第k_rp个可达点的PrX的分子
    end
    sumK=sum(PrX01_eq1);      %ref2论文中是一个归一化常数K，用的所有PrX0_eq1的和，因此是sumK
    % 得到一个列向量，是第k_theta个theta下所有点的PrX值
    PrX1=PrX01_eq1/sumK;  %可达点k_rp在k_theta下的公式（1）的值
    %保存给下一个时刻公式（2）用
    PrX_eq11=[PrX_eq11,PrX1];    %每次往后补一列，这一列是第k_theta个theta下所有点的PrX值
end
t12=toc;
t1=t12-t11
t21=toc;
for k_theta=1:1:6

    PrX02=exp(-alpha_infer*(RP_all_L1(:,k_theta)-Theta_L0(k_theta)));
    sumK=sum(PrX02);      %ref2论文中是一个归一化常数K，用的所有PrX0_eq1的和，因此是sumK
    % 得到一个列向量，是第k_theta个theta下所有点的PrX值
    PrX2=PrX02/sumK;  %可达点k_rp在k_theta下的公式（1）的值
    %保存给下一个时刻公式（2）用
    PrX_eq12=[PrX_eq12,PrX2];    %每次往后补一列，这一列是第k_theta个theta下所有点的PrX值
end
t22=toc;
t2=t22-t21