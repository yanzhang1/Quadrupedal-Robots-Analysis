function x0 = fixedpoint_2
%state =[y theta xdot thetadot]
a1=1;  %m
a2=5.5;%m/s
 %deg
%0.455530934771    
u_0=0.145*pi;%rad                   
h= 1e-6;
x0= [a1 a2];

i=1;
dx=[1 1];
while max(abs(dx)) > 1e-4
%1 by 4 matrix   
%'P_m' function is used to describe the Poincare map
dPdy=(P_m([x0(1)+h/2,x0(2)],u_0)-P_m([x0(1)-h/2,x0(2)],u_0))/h;
dPdxdot=(P_m([x0(1),x0(2)+h/2],u_0)-P_m([x0(1),x0(2)-h/2],u_0))/h;
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdxdot.'];

%4 by 1 matrix
M = (inv(eye(2) - Jacobian_P));
dx = M*(P_m(x0,u_0).'-x0.');
x0 = x0 + dx.';
i=i+1;
end


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