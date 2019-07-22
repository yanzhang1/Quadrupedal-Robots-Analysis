function FixedPointSet = Get_Fixed_Points(threshold,u_0)
tic;

GoodOnes = Generate_Good_ICs(threshold,u_0);
[a,~] = size(GoodOnes);

FixedPointSet = zeros(a,4);

for k = 1:a
    FixedPointSet(k,1:4) = Newton_Raphson(GoodOnes(k,1:4),u_0) ; 
    fprintf('The fixed point is [%.6f\t %.6f\t %.6f\t %.6f\t] \n',FixedPointSet(k,1:4));
end
toc;

end