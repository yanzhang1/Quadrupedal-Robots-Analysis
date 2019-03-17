function PP = Prop(u_0,N,R,threshold)
count = 0;
for j = 1:N

    [P,~] =  step1(100,threshold,u_0,R);

    if P ~= 0
        count = count +1 ;
    end
    disp(j);
end
   PP = count/N;
end