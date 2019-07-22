function GoodExamples = Generate_Good_ICs(threshold,u_0)

N = 30000;
mu = [1 1 1];
sigma = [1 1 1];
Valid_ICs = zeros(N,4); %the valid initial condition of Poincare Map
m = 0;
x0 = zeros(1,4);
range = zeros(4,2);

for j = 1:N

    x0(1,1) = lognrnd(mu(1),sigma(1));
    x0(1,2) = -pi/2 + pi*rand(1);
    x0(1,3) = lognrnd(mu(2),sigma(2));
    x0(1,4) = lognrnd(mu(3),sigma(3));
    x1 = Poincare_Map(x0,u_0);

      if norm((x1-x0),inf) <= threshold
            m = m + 1;
            Valid_ICs(m,1:4) = x0; 
      end
      P = m/N;
      
      if P == 0 
            mu = [rand(1) rand(1) rand(1)]; 
      end
            disp(m);disp('/');disp(j);disp('/');disp(N);
     
end  

for k = 1:4
   range(k,:) = [min(Valid_ICs(1:m,k)) max(Valid_ICs(1:m,k))];
   if range == zeros(4,2)
       fprintf('Not valid. Try again!!');
       break
   end
end

N = 4000;
threshold2 = (threshold:-0.1:0.1);
R = range;
x_0 = zeros(1,4);
for h = 1:length(threshold2)
n=0;
    for i = 1:N
        
        for z = 1:4
            x_0(1,z) =  R(z,1) + (R(z,2)-R(z,1))*rand(1);
        end
        
        x1 = Poincare_Map(x_0,u_0);

        if norm(x1-x_0) <= threshold2(h)
            n = n+1;
            VS(n,1:4) = x_0;
          
        end
         disp(n);disp('/');disp(h);disp('/');disp(i);disp('/');disp(N)     
       
    end
    for w = 1:4
            R(w,:) = [min(VS(1:n,w))  max(VS(1:n,w))];   
    end
end

GoodExamples = VS(1:n,1:4);
    
end