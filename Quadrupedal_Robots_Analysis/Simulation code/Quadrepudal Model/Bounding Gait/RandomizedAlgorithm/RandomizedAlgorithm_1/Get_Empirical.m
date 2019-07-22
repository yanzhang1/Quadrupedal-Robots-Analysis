function empirical_mean = Get_Empirical(InitialConditionSet,n,u_0)
%n=16;
m=length(InitialConditionSet);
empirical_mean = zeros(n,3); % each row consists of two gains and one result
performance = zeros(m,1);
for i = 1:n
    
    u(1) = u_0(1) -0.01 + 0.02*rand(1);
    u(2) = u_0(2) -0.01 + 0.02*rand(1);
    for j = 1:m
        q_0 = InitialConditionSet(j,:);    
        x_new = Poincare_Map(q_0,u);
        performance(j) = norm((x_new-q_0),inf);
    end
    empirical_mean(i,1:2) = u;
    empirical_mean(i,3) = mean(performance);
    
end

end