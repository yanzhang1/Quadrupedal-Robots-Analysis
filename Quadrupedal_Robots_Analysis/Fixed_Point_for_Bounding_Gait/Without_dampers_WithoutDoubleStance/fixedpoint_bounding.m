function x0 = fixedpoint_bounding(a1,a2,a3,a4,u_0)
%state =[y theta xdot thetadot]
 
%u_0=[0.226892803,0.191986218];%rad                   
h= 1e-6;
x0= [a1 a2 a3 a4];

i=1;
dx=[1 1 1 1];

while max(abs(dx)) > 1e-4
%1 by 4 matrix  
tic; 
%'P_bounding' function is used to describe the Poincare map
dPdy=(P_bounding([x0(1)+h/2,x0(2),x0(3),x0(4)],u_0)-P_bounding([x0(1)-h/2,x0(2),x0(3),x0(4)],u_0))/h;
dPdtheta=(P_bounding([x0(1),x0(2)+h/2,x0(3),x0(4)],u_0)-P_bounding([x0(1),x0(2)-h/2,x0(3),x0(4)],u_0))/h;
dPdxdot=(P_bounding([x0(1),x0(2),x0(3)+h/2,x0(4)],u_0)-P_bounding([x0(1),x0(2),x0(3)-h/2,x0(4)],u_0))/h;
dPdthetadot=(P_bounding([x0(1),x0(2),x0(3),x0(4)+h/2],u_0)-P_bounding([x0(1),x0(2),x0(3),x0(4)-h/2],u_0))/h;
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdtheta.'  dPdxdot.'  dPdthetadot.'];
toc;
%4 by 1 matrix
M = (inv(eye(4) - Jacobian_P));
dx = M*(P_bounding(x0,u_0).'-x0.');
x0 = x0 + dx.';
i=i+1;
end


fprintf('The Corresponding Jacobian Matrix is:\n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f \n%6.6f  %6.6f %6.6f  %6.6f ',...
Jacobian_P);
fprintf('The e-values of Jacobian are : %.6f\t %.6f\t %.6f\t %.6f\t  \n',...
    eig(Jacobian_P))
fprintf('The fixed point is [%.6f\t %.6f\t %.6f\t %.6f\t]; The iteration is %d \n',x0,i)

stable = (abs(eig(Jacobian_P)) < 1);
   if stable
      fprintf('The system is stable.\n');
   else
      fprintf('WARNING:  The system is NOT stable.\n');
   end



end