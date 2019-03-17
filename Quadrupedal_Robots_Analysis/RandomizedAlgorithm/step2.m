function [range,matrix1,state,norm_value,good_examples] = step2(N,R,u_0)
tic;
VS = zeros(N,4);
threshold = [1 0.5 0.2 0.1 0.05 0.02 0.01 ];
good_examples = zeros(7,1);
for jj = 1:length(threshold)
     m=0;
    for i = 1:N
       
        a1 = R(1,1) + (R(1,2)-R(1,1))*rand(1); % define the range of initial value 
        a2 = R(2,1) + (R(2,2)-R(2,1))*rand(1);
        a3 = R(3,1) + (R(3,2)-R(3,1))*rand(1);
        a4 = R(4,1) + (R(4,2)-R(4,1))*rand(1);

        x0 = [a1 a2 a3 a4];
        x1 = allmo(x0,u_0);

        if norm(x1-x0) <= threshold(jj)
      
         m = m+1;
          VS(m,1:4) = x0;
          
        end     
         disp(m);disp('/');disp(i);disp('/');disp(N)     %        break
       
    end
   % P = m/N;
    disp(jj);
    range = [min(VS(1:m,1))  max(VS(1:m,1));    
             min(VS(1:m,2))  max(VS(1:m,2));
             min(VS(1:m,3))  max(VS(1:m,3));
             min(VS(1:m,4))  max(VS(1:m,4))];
    R = range;
    matrix1(4*jj-3:4*jj,1:2) = range; 
    good_examples(jj,1) = m;
end    
    state = VS(1:m,1:4);
    for kk = 1:m
        norm_value(kk,1) = norm(VS(kk,1:4)- allmo(VS(kk,1:4) , u_0));
        
    end

toc;
   

end