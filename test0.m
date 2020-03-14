clc;
clear;
N=10;
true_select=zeros(1,N);
A=zeros(1,100);
for a=1:N
PP=[0.1 0.2 0.1 0.15 0.1 0.05 0.05 0.05 0.05 0.05 0.05 0.05];
Pcum=zeros(1,length(PP));
Pcum(1)=PP(1);
for i=2:length(PP)
    Pcum(i)=Pcum(i-1)+PP(i);
end
b=a
Select=find(Pcum>=rand)
true_select(a)=Select(1)
end
% clc;
% clear;
% true_select=zeros(1,100)
% A=zeros(1,100)
% for a=1:100
% PP=[0.1 0.2 0.7]
% Pcum=zeros(1,3)
% Pcum(1)=PP(1)
% for i=2:3
%     Pcum(i)=Pcum(i-1)+PP(i);
% end
% Select=find(Pcum>=rand)
% true_select(a)=Select(1)
% end