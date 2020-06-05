function [Rb_points,PrX_eq1] = BayesEqu1( Boat_x,Boat_y,L0_paths,Boat_theta,speed,Theta,Theta_end,FM_map,MapSize,Res)
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
    L0 = rot90(L0',2);
    L0_integral = RiskIntegral(L0,FM_map);
    % �õ�ÿһ��theta��Ӧ�ĵ�ǰ��L0���ջ���ֵ
    Theta_L0=[Theta_L0,L0_integral];
end
t_res=ceil(Res/speed); %Ҫ��֤ÿ��������ǰ��1��
%% ����1.2. �ҳ�tʱ�����еĿɴ��Reachable points(Rb_points)
% ��ɸѡAG_points�ķ���ȷ��ĳһ��ʱ�̵Ŀɴ�㼯�ϣ�r=V*(t-1),R=V*t��
Rb_points=[];
for RR_t=1:1:10   %ÿ�β²�10��
    r_infer=0;
    R_infer=RR_t*speed*t_res;
    alpha=45;
    %RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
    RRpoint0=ReachableRange(Boat_x,Boat_y,Boat_theta,alpha,R_infer,r_infer,MapSize,Res);
    %˳�����½����n*2�ľ�����˲����ڳߴ粻ͳһ�����⣬���õ�l*2�ľ���lΪ����3�����ڿɴ��ĸ���
    Rb_points=[Rb_points;RRpoint0];
end
% ɾȥ��ͬ�ĵ㣨����ͬ���У�
% ʹ��unique����
Rb_points=unique(Rb_points,'rows','stable');
% unique(A,��stable��)ȥ�ظ�������Ĭ�ϵ�������unique(A,��sorted��)����sorted��һ��ʡ�Ե��ˡ�
Rb_points=fliplr(Rb_points);

%% ����1.3. ����L1�����߻���
% ��ÿһ��RR_point�ҵ���Ӧ��TS-Reachable-theta��·��L1
RP_all_L1=[];
for k_rp=1:1:size(Rb_points,1)
    Reach_point_now=Rb_points(k_rp,:);
    %RR_points�ͺ�������һ�����õ����ǵ�ͼ�ϵ�ĺ��������n*2�ľ���
    start_point(1,1) = Reach_point_now(1);
    start_point(1,2) = Reach_point_now(2);
    %TODO �����Ƿ���Ҫ���������
    [~, L1_paths] = FMM(FM_map,start_point',Theta_end');
    
    %�ظ�һ���L0�ļ��㣬ֻ���������Reach_point
    RP_L1=[];    %���ÿһ�������theta
    for k_theta=1:1:size(Theta,1)     %���ÿһ��theta
        %��L0���ݴ���,���ջ�������դ������ģ����еı�Ҷ˹�ƶ϶�����դ�������
        L1 = L1_paths{k_theta};
        L1=rot90(L1',2);
        %�õ����ǵ�ǰ����Ե�ǰTheta��L1����ֵ
        L1_integral=RiskIntegral(L1,FM_map);
        %�õ����ǵ�ǰ���������Theta��L1����ֵ��������
        RP_L1=[RP_L1,L1_integral];
    end
    %�õ�ÿһ�д���һ���ɴ��(RP,reach points)��ÿһ����һ��theta��L1
    % RP_all_L1=[RP1Theta1,RP1Theta2,RP1Theta3,RP1Theta4,RP1Theta5,RP1Theta6
    %            RP2Theta1,RP2Theta2,RP2Theta3,RP2Theta4,RP2Theta5,RP2Theta6
    %            RP3Theta1,RP3Theta2,RP3Theta3,RP3Theta4,RP3Theta5,RP3Theta6
    %            RP4Theta1,RP4Theta2,RP4Theta3,RP4Theta4,RP4Theta5,RP4Theta6
    %            RP5Theta1,RP5Theta2,RP5Theta3,RP5Theta4,RP5Theta5,RP5Theta6];
    RP_all_L1=[RP_all_L1;RP_L1];
end

%% ����1.4. ���ÿһ��theta�����µĹ�ʽ��1��
PrX_eq1=[];
alpha_infer=1;
for k_theta=1:1:size(Theta,1)
    PrX0=exp(-alpha_infer*(RP_all_L1(:,k_theta)-Theta_L0(k_theta)));
    sumK=sum(PrX0);      %ref2��������һ����һ������K���õ�����PrX0_eq1�ĺͣ������sumK
    % �õ�һ�����������ǵ�k_theta��theta�����е��PrXֵ
    PrX=PrX0/sumK;  %�ɴ��k_rp��k_theta�µĹ�ʽ��1����ֵ
    % �������һ��ʱ�̹�ʽ��2����
    % ÿ������һ�У���һ���ǵ�k_theta��theta�����е��PrXֵ
    % ���PrX_eq1��ÿһ�к�Ϊ1
    PrX_eq1=[PrX_eq1,PrX];
end

end

