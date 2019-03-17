function x0 = fixedpoint(u_0)
%state =[y  xdot]
a1=0.35;  %m       fixedpoint1 =  [1.049,0.9],0.026*pi
a2=1;     %m/s         fixedpoint2 =  [1.049,0.9],0.06667*pi
 %deg
%0.455530934771    
%u_0=0.089*pi;%8rad                   
h= 1e-6;
x0= [a1 a2];

dx=[1 1];
i=1;
tic;
%% 

while max(abs(dx)) > 1e-5
%1 by 4 matrix   
%'P_m' function is used to describe the Poincare map
dPdy=(P_pronk([x0(1)+h/2,x0(2)],u_0)-P_pronk([x0(1)-h/2,x0(2)],u_0))/h;
dPdxdot=(P_pronk([x0(1),x0(2)+h/2],u_0)-P_pronk([x0(1),x0(2)-h/2],u_0))/h;
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdxdot.'];

%4 by 1 matrix
M = (inv(eye(2) - Jacobian_P));
dx = M*(P_pronk(x0,u_0).'-x0.');
x0 = x0 + dx.';
i=i+1;
end
%% 
toc;
fprintf('The Corresponding Jacobian Matrix is:\n%6.6f  %6.6f   \n%6.6f  %6.6f \n ',...
Jacobian_P);
fprintf('The e-values of Jacobian are : %.6f\t %.6f\t \n',...
    eig(Jacobian_P))
fprintf('The fixed point is [%.6f\t %.6f\t ]; The iteration is %d \n',x0,i)

stable = (abs(eig(Jacobian_P)) < 1);
   if stable
      fprintf('The system is stable.\n');
   else
      fprintf('WARNING:  The system is NOT stable.\n');
   end
end
