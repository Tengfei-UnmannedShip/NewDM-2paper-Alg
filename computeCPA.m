function CPA = computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,time)
%% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
%��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
V_x1 = v_os*sind(course_os);%WTF:sind���ԽǶ�Ϊ�Ա�����sinֵ��sin���Ի���Ϊ��λ�ģ�deg2rad���Ƕ�ת��Ϊ����
V_y1 = v_os*cosd(course_os);

V_x2 = v_ts*sind(course_ts);
V_y2 = v_ts*cosd(course_ts);
%WTF:����������ٶ�
V_x = V_x1-V_x2;
V_y = V_y1-V_y2;  %WTF:��������ʾ��Ŀ�괬Ϊ���������������ٶ�

pos = pos_ts-pos_os;%WTF:������λ���������ɱ���ָ��Ŀ�괬

%WTF:��������̾���
p_x = [V_y*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2) -V_x*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2)];

d = norm(p_x-pos,2);
%% WTF:λ���ж��㷨
if V_x*pos(1)+V_y*pos(2)<=0 %˵��������Զ��
%WTF:����pos=pos_target-pos_own,����ٶ�����v=(V_x,V_y),��˻�pos?v=V_x*pos(1)+V_y*pos(2)������н�(0,pi),
%WTF:���<=0,���������λ��������Ŀ�괬����ٶ������нǴ���90�ȣ���������Զ�룬��������Զ��ʱ����С����DCPA��Ϊ��ǰʱ�̵�ֵ
   TCPA = 0;
else
    TCPA = d/sqrt(V_x^2+V_y^2);
end
if TCPA>time
    TCPA=time;
end
%% WTF:�������
pos1=[pos_os(1)+v_os*sind(course_os)*TCPA, pos_os(2)+v_os*cosd(course_os)*TCPA];
pos2=[pos_ts(1)+v_ts*sind(course_ts)*TCPA, pos_ts(2)+v_ts*cosd(course_ts)*TCPA];

DCPA=norm(pos1-pos2,2);%WTF:norm�������ķ�������(x1^2+x2^2+x3^2)^(1/2)�����˴��������ķ���

%�������:��������㴦OS����pos1��TS����pos2��DCPA��TCPA
CPA = [pos1,pos2,DCPA,TCPA]; 
end