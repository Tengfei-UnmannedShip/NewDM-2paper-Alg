function L0_integral = RiskIntegral( L0,RiskMap )
% RISKINTEGRAL���ڼ���դ��·���Ϸ��յ��߻���

x0=L0(1:50,1); %�滮��·��ƽ������
y0=L0(1:50,2);
%ƽ������
y_new=smooth(y0);
x_new=smooth(x0);
x_new1=smooth(x_new);
y_new1=smooth(y_new);

L0(1:50,1)=x_new1;
L0(1:50,2)=y_new1; %�ó����յ�ƽ�����L0
% ����ÿһ��L0�ķ��ջ���InL0
% 1)L0���������ȡ�����ڵ�դ�����꣬ɾȥ�ظ��ģ���ֹһ������������
L0_point0=floor(L0);    %L0���������ȡ�����ڵ�դ������
L0_point=unique(L0_point0,'rows','stable');  %ɾȥ�ظ��У�����ԭ˳��
% 2)����ÿһ�����ӵķ���ֵ������L0�ķ��ջ���
L0_integral=0;
for k_L0=1:1:size(L0_point,1)
    m_L0=L0_point(k_L0,1);
    n_L0=L0_point(k_L0,2);
    L0_integral=L0_integral+RiskMap(m_L0,n_L0);
end

end

