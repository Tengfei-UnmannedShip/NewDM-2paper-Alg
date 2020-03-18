for i=1:1:4
    A(i).b=rand(3,3);
    
end
for k=1:1:4
if k==2
   A(k).c=10*rand(2,2); 
elseif k==3
   A(k).c=floor(10*rand(2,2)); 
    
end
end