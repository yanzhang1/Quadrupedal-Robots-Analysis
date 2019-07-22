function x0 = fixedpoint_bounding111(a1 ,a2, a3, a4, u_0)
%state =[y theta xdot thetadot]
  
%u_0=[0.226892803,0.191986218];%rad          
h= 1e-6;
x0= [a1 a2 a3 a4];


i=1;
dx=[1 1 1 1];

while max(abs(dx)) > 1e-5
%1 by 4 matrix  
tic; 

x0 = [p(a1) p(a2) p(a3) p(a4)];
%'P_bounding4' function is used to describe the Poincare map
dPdy=P1(x0);

if isnan(dPdy) == 1
    while isnan(dPdy) == 1
    x0 = [p(a1) p(a2) p(a3) p(a4)];
    dPdy=P1(x0);
    end
elseif isnan(dPdy) == 0
    continue
end
dPdtheta=P2(x0);
dPdxdot=P3(x0);
dPdthetadot=P4(x0);
%4 by 4 matrix
Jacobian_P= [dPdy.'  dPdtheta.'  dPdxdot.'  dPdthetadot.'];
toc;
%4 by 1 matrix
M = (inv(eye(4) - Jacobian_P));
dx = M*(P_bounding4(x0,u_0).'-x0.');
x0 = x0 + dx.';
i=i+1;
%end


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

    function m = p(q)
        beta_1 = rand(1);
        beta_2 = rand(1);
        beta_3 = rand(1);
        beta_4 = rand(1);
        
        m = beta_1*q^4 + beta_2*q^3 +beta_3*q^2 +beta_4*q ;
        
    end

    function dPdy = P1(x0)
        dPdy = (P_bounding4([x0(1)+h/2,x0(2),x0(3),x0(4)],u_0)-P_bounding4([x0(1)-h/2,x0(2),x0(3),x0(4)],u_0))/h;  
    end

    function dPdtheta = P2(x0)
        dPdtheta = (P_bounding4([x0(1),x0(2)+h/2,x0(3),x0(4)],u_0)-P_bounding4([x0(1),x0(2)-h/2,x0(3),x0(4)],u_0))/h;
    end

    function dPdxdot = P3(x0)
        dPdxdot = (P_bounding4([x0(1),x0(2),x0(3)+h/2,x0(4)],u_0)-P_bounding4([x0(1),x0(2),x0(3)-h/2,x0(4)],u_0))/h;
    end

    function dPdthetadot = P4(x0)
        dPdthetadot = (P_bounding4([x0(1),x0(2),x0(3),x0(4)+h/2],u_0)-P_bounding4([x0(1),x0(2),x0(3),x0(4)-h/2],u_0))/h;
    end

end