%用于显示测试航向角等参数的变化率
for i=1:4
    
    tDec=[];
    for k=1:1:size(Boat(i).Dechis,2)
    %提取决策点
    tDec=[tDec;Boat(i).Dechis(k).data(1)];
    end
    a=Boat(i).HisCOG(1:end-1,2);
    a0=Boat(i).HisCOG(2:end,2);
    b=a0-a;
    b1=b(tDec);
    b2=Boat(i).HisCOG(tDec,2);
    figure
    subplot(2,1,1)
    plot(Boat(i).HisCOG(:,2))
    hold on
    plot(tDec,b2,'rs')
    ylabel('航向角');
    xlabel('时刻');
    title(['船',num2str(i),'的角度变化曲线'])
    subplot(2,1,2)
    plot(b)
    hold on
    plot(tDec,b1,'rs')
    upline=10;
    bottomline=-10;
    xrange=1:1:size(b,1);
    plot(xrange,upline,'r.-')
    plot(xrange,bottomline,'r.-')

    ylabel('航向角变化幅度');
    xlabel('时刻');
    title(['船',num2str(i),'航向角变化幅度曲线'])
end
