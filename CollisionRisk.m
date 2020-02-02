function CurrentRisk= CollisionRisk(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,d_thre)
% 通过DCPA和TCPA判断是否有碰撞风险
% d_thre = 3*1852;  %d_thre为风险阈值
time=2500;

CAP=computeCPA(v_os,course_os,pos_os,v_ts,course_ts,pos_ts,time);
DCPA=CAP(5);
TCPA=CAP(6);
if DCPA<=d_thre && TCPA>1   %TCPA>1即排除啊当前位置为CPA的情况
    CurrentRisk=1; %有碰撞风险为1
else
    CurrentRisk=0;
end

end

