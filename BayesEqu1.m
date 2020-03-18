function [RR_points,PrX_eq1] = BayesEqu1( Boat_x,Boat_y,L0_paths,Boat_theta,speed,Theta,Theta_end,FM_map,MapSize,Res)
% ��Ҷ˹�ƶϵĹ�ʽ1�����ݵ�ǰ��λ�ú�theta������һ����λ�÷ֲ�
% ���룺Boat_x,Boat_y,Boat_theta,speed������TS����״̬
%      Theta����0���ó���theta�������͸���
%      MapSize,Res����ͼ��Ϣ
% �����RR_points����һ��ʱ�����еĿɴ�㣬դ����ʽ����ͼ��ÿһ�������
%      PrX_eq1��PrX��ֵ��ÿһ�д���һ���ɴ��RR_points��ÿһ�д���һ��theta
% ����1.   ��ʽ(1)���µ�ǰλ�÷ֲ�
%% ����1.1. ����L0��������ջ���
% �ҵ�TS����㵽theta��·��L0
Theta_L0=[];
for k_theta=1:1:size(Theta,1)     %���ÿһ��theta
    %��L0���ݴ���,���ջ�������դ������ģ����еı�Ҷ˹�ƶ϶�����դ�������
    L0 = L0_paths{k_theta};
    L0=rot90(L0',2);
    L0_integral=RiskIntegral(L0,FM_map);
    
    Theta_L0=[Theta_L0;L0_integral];
end

t_res=ceil(Res/speed); %Ҫ��֤ÿ��������ǰ��1��

%% ����1.2. �ҳ�tʱ�����еĿɴ��Reachable
% ��ɸѡAG_points�ķ���ȷ��ĳһ��ʱ�̵Ŀɴ�㼯�ϣ�r=V*(t-1),R=V*t��
r_infer=0;
R_infer=speed*t_res;
alpha=30;
%RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
RR_points=ReachableRange(Boat_x,Boat_y,Boat_theta,alpha,R_infer,r_infer,MapSize,Res);

%% ����1.3. ����L1�����߻���
% ��ÿһ��RR_point�ҵ���Ӧ��TS-Reachable-theta��·��L1
Reachs_L1=[];
for k_reach=1:1:size(RR_points,1)
    Reach_point=RR_points(k_reach,:);
    %RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
    start_point(1,2) = Reach_point(1);
    start_point(1,1) = Reach_point(2);
    [~, L1_paths] = FMM(FM_map,start_point',Theta_end');
    %�ظ�һ���L0�ļ��㣬ֻ���������Reach_point
    Reach_L1=[];    %���ÿһ�������theta
    Reachs_L1=[];   %������е������theta
    for k_theta=1:1:size(Theta,1)     %���ÿһ��theta
        %��L0���ݴ���,���ջ�������դ������ģ����еı�Ҷ˹�ƶ϶�����դ�������
        L1 = L1_paths{k_theta};
        L1=rot90(L1',2);
        %�õ����ǵ�ǰ����Ե�ǰTheta��L1����ֵ
        L1_integral=RiskIntegral(L1,FM_map);
        %�õ����ǵ�ǰ���������Theta��L1����ֵ��������
        Reach_L1=[Reach_L1,L1_integral];
    end
    %�õ�ÿһ�д���һ���ɴ�㣬ÿһ����һ��theta��L1
    Reachs_L1=[Reachs_L1;Reach_L1];
end

%% ����1.4. ���ÿһ��theta�����µĹ�ʽ��1��
PrX_eq1=[];
PrX0_eq1=[];
alpha_infer=1;
for k_theta=1:1:size(Theta,1)
    for k_reach=1:1:size(RR_points,1)
        PrX0=-alpha_infer*(Reachs_L1(k_reach,k_theta)-Theta_L0(k_theta));
        PrX0=exp(PrX0);
        PrX0_eq1=[PrX0_eq1,PrX0];   %ÿһ������һ������������ǵ�k_reach���ɴ���PrX0
    end
    PrX_temp=[];
    for k_reach=1:1:size(RR_points,1)
        PrX=PrX0_eq1(k_reach)/sum(PrX0_eq1);  %�ɴ��k_reach��k_theta�µĹ�ʽ��1����ֵ
        % �õ�һ�����������ǵ�k_theta��theta�����е��PrXֵ
        PrX_temp=[PrX_temp;PrX];   %ÿ�����²�һ����ֵΪ��k_reach���ɴ���ڵ�k_theta��theta�µ�PrXֵ
    end
    %�������һ��ʱ�̹�ʽ��2����
    PrX_eq1=[PrX_eq1,PrX_temp];    %ÿ������һ�У���һ���ǵ�k_theta��theta�����е��PrXֵ
end

end

