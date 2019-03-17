function x_star = fixedpoint_bounding21(x0,u_0)
%state =[y theta xdot thetadot]

%u_0=[0.226892803,0.191986218];%rad                   
h= 1e-12;

i =0;

dx=[1 1 1 1];

while max(abs(dx)) > 1e-6
%1 by 4 matrix  
%tic; 
%'P_bounding' function is used to describe the Poincare map
dPdy=(allmo([x0(1)+h/2,x0(2),x0(3),x0(4)],u_0)-allmo([x0(1)-h/2,x0(2),x0(3),x0(4)],u_0))/h;
dPdtheta=(allmo([x0(1),x0(2)+h/2,x0(3),x0(4)],u_0)-allmo([x0(1),x0(2)-h/2,x0(3),x0(4)],u_0))/h;
dPdxdot=(allmo([x0(1),x0(2),x0(3)+h/2,x0(4)],u_0)-allmo([x0(1),x0(2),x0(3)-h/2,x0(4)],u_0))/h;
dPdthetadot=(allmo([x0(1),x0(2),x0(3),x0(4)+h/2],u_0)-allmo([x0(1),x0(2),x0(3),x0(4)-h/2],u_0))/h;
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdtheta.'  dPdxdot.'  dPdthetadot.'];
%toc;
%4 by 1 matrix
M = (inv(eye(4) - Jacobian_P));
dx = M*(allmo(x0,u_0).'-x0.');
x0 = x0 + dx.';
i = i+1;
end

x_star = x0;
%fprintf('The Corresponding Jacobian Matrix is:\n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f \n%6.6f  %6.6f %6.6f  %6.6f \n',...
%Jacobian_P);
%fprintf('The Corresponding transpose Jacobian Matrix is:\n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f  \n%6.6f  %6.6f %6.6f  %6.6f \n%6.6f  %6.6f %6.6f  %6.6f \n',...
%Jacobian_P');
%fprintf('The determinant of Jacobian is:%6.6f\n',det(Jacobian_P));
%fprintf('The e-values of Jacobian are : %.6f\t %.6f\t %.6f\t %.6f\t  \n',...
%    eig(Jacobian_P))
%fprintf('The fixed point is [%.6f\t %.6f\t %.6f\t %.6f\t]; \n The
%iteration is %d \n',x0,i)

%stable = (abs(eig(Jacobian_P)) < 1);
%   if stable
%      fprintf('The system is stable.\n');
%   else
%      fprintf('WARNING:  The system is NOT stable.\n');
%   end



end