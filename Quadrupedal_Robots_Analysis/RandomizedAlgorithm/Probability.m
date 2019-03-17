function Fixedpoint = Probability(R,u_0,yita,beta)
%count =0;
%tic;
N = (1/(2*yita^2))*log(2/beta); 

Fixedpoint = zeros(1,4);
x0 = zeros(1,4);%Range matrix of state value [ Range Guess associated with physical meaning and reality ]
%Record = zeros(round(N),4);
for i = 1:round(N)
    
    for p = 1:4
       x0(1,p) = R(p,1) + (R(p,2) - R(p,1))*rand(1); 
       %x0(1,p) = x(1,p);
    end
 
 x_star =fixedpoint_bounding21(x0,u_0);
 
 if norm((x_star-x0),inf) <= 2
  %  count = count + 1;
    Fixedpoint = x_star;
    break
    
 end
 disp(i);disp(N)
%disp(i);disp(N);
end
%toc;
%P = count/N;
end