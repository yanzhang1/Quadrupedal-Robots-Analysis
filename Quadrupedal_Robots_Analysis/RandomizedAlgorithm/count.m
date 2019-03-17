function PP = count(N,threshold,u_0,R)
m=0;
for i = 1:2000
[~,~,P] = step111(N,threshold,u_0,R);
if P~= 0
m = m+1;
end
disp(i)
end
PP = m/2000;
end