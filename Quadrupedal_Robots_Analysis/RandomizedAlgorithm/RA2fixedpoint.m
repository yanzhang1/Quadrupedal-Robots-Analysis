function x_star = RA2fixedpoint(threshold_step1,N_1,N_2,u_0,R_approx)
tic;

threshold = threshold_step1;
N = N_1;
R = R_approx;
[range1,~] = step1(N,threshold,u_0,R);
R = range1;
N = N_2;
[~,~,state,~,~] = step2(N,R,u_0);
[a,~] = size(state);
x_star = zeros(a,4);
for kk = 1:a
    x_star(kk,1:4) = NewtonRaphson(state(kk,1:4),u_0) ; 
    fprintf('The fixed point is [%.6f\t %.6f\t %.6f\t %.6f\t] \n',x_star(kk,1:4));
end
toc;

end