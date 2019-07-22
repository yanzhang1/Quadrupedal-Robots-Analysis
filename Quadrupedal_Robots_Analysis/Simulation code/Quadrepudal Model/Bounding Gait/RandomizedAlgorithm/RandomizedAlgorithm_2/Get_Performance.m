function objective_function = Get_Performance(u_0,GoodOnes,Kp,Kd)
%u_0 is the good touchdown angles which maxmize the performance. It's already obtained by Randomized Algorithm 1 
m = length(GoodOnes);
objective_function = zeros(m,1);

    for i= 1:m
        
        disp(i);
        q_0 = GoodOnes(i,:);
        performance = Poincare_Map2(q_0,u_0,Kp,Kd);
        M_function = performance;
       
        if performance > 10
            objective_function(i) = 1;
        else 
            objective_function(i) = M_function/(M_function + 1);
        end
    
    end

end