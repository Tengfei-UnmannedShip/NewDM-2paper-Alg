%������ʾ���Ժ���ǵȲ����ı仯��
for i=1:4
    
    tDec=[];
    for k=1:1:size(Boat(i).Dechis,2)
    %��ȡ���ߵ�
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
    ylabel('�����');
    xlabel('ʱ��');
    title(['��',num2str(i),'�ĽǶȱ仯����'])
    subplot(2,1,2)
    plot(b)
    hold on
    plot(tDec,b1,'rs')
    upline=10;
    bottomline=-10;
    xrange=1:1:size(b,1);
    plot(xrange,upline,'r.-')
    plot(xrange,bottomline,'r.-')

    ylabel('����Ǳ仯����');
    xlabel('ʱ��');
    title(['��',num2str(i),'����Ǳ仯��������'])
end
