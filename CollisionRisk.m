function CurrentRisk= CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre)
% ͨ��DCPA��TCPA�ж��Ƿ�����ײ����
% d_thre = 3*1852;  %d_threΪ������ֵ
time=2500;

CAP=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,time);
DCPA=CAP(5);
TCPA=CAP(6);
if DCPA<=d_thre && TCPA>1   %TCPA>1���ų�����ǰλ��ΪCPA�����
    CurrentRisk=1; %����ײ����Ϊ1
else
    CurrentRisk=0;
end

end

