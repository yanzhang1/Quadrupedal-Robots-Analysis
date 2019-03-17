function Fixedpoint = get_fixed_point(N,threshold,u_0,R,yita,beta)

tic;
[range,solution] = step1(N,threshold,u_0,R);
R1  = range;
if solution == zeros(N,4)
    fprintf("oh no");
    error("hehe");
end
Fixedpoint = Probability(R1,u_0,yita,beta);
toc;




end