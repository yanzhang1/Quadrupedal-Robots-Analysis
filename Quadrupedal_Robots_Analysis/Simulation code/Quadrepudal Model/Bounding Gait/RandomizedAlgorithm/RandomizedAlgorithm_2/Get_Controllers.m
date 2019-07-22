function [gains,results]=Get_Controllers(GoodOnes,u_0)
%need to fix the touchdown angle u_0

n=25;
gains = zeros(n,2);
results = zeros(n,1);


for j = 1:n
    
    Kp = 40 + 20*rand(1);
    Kd = -10 + 20*rand(1);

    gains(j,:) = [Kp,Kd];
    objective_function = Get_Performance(u_0,GoodOnes,Kp,Kd);
    results(j) = mean(objective_function);

    disp('times');disp('is');disp(j);
    
end

end