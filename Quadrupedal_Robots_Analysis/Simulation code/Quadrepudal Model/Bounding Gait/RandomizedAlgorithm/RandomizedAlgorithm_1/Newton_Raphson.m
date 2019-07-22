function x_star = Newton_Raphson(x0,u_0)              
h= 1e-12;
i =0;
dx=[1 1 1 1];

while max(abs(dx)) > 1e-6
%1 by 4 matrix  
tic; 
%'P_bounding' function is used to describe the Poincare map
dPdy=(allmo([x0(1)+h/2,x0(2),x0(3),x0(4)],u_0)-allmo([x0(1)-h/2,x0(2),x0(3),x0(4)],u_0))/h;
dPdtheta=(allmo([x0(1),x0(2)+h/2,x0(3),x0(4)],u_0)-allmo([x0(1),x0(2)-h/2,x0(3),x0(4)],u_0))/h;
dPdxdot=(allmo([x0(1),x0(2),x0(3)+h/2,x0(4)],u_0)-allmo([x0(1),x0(2),x0(3)-h/2,x0(4)],u_0))/h;
dPdthetadot=(allmo([x0(1),x0(2),x0(3),x0(4)+h/2],u_0)-allmo([x0(1),x0(2),x0(3),x0(4)-h/2],u_0))/h;
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdtheta.'  dPdxdot.'  dPdthetadot.'];
toc;
%4 by 1 matrix
M = (inv(eye(4) - Jacobian_P));
dx = M*(allmo(x0,u_0).'-x0.');
x0 = x0 + dx.';
i = i+1;
end

x_star = x0;

end