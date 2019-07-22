function [alpha3,delta,alpha6] = BezierPolynomial_FS(z,q_0)
%%
g=9.8;                         %gravity acceleration 
k=3520;                         %spring constant
m=20.865;                        %torso mass
l_0=0.323;                       %rest length of leg
L = 0.276;                       % half length of torso
I = 1.3;                         % Torso moment of inertia

options_front_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_front_stance);

t0 = 0;
time= 10;

[~,q] =ode45(@front_stance_motion, [t0,time] , q_0,options_front_stance);

delta = q(end,2) + q(end,3) - q_0(2) - q_0(3);

s3 =( q(:,3)+q(:,2) -q_0(3) - q_0(2))/delta;

n = length(q(:,2));
 
A3 = zeros(n,z-1);

    for i = 1:n 
        for j = 1:z
          A3(i,j) = ( factorial(z-1)/(factorial(j-1)*factorial(z - j))) * s3(i)^(j-1) *(1-s3(i))^(z-j) ; 
        %A3(i,j) = s3(i).^(j-1);
        end
    end
    

    alpha3 = pinv(A3)*(q(:,3));
  
    alpha6 = pinv(A3)*q(:,6);
 

function dqdt=front_stance_motion(~,q)

u =0;

dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4=( - k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u)/q(1)    )/I  + L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
dqdt_5=(k*L*cos(q(2))*(q(1)-l_0)*(q(1)+L*sin(q(2))))/(I*q(1))  + ( ((q(1)+L*sin(q(2)))^2)/(I*q(1)^2)  + 1/(m*(q(1)^2)))*u  + (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
dqdt_6= ( -(k*L*cos(q(2))*(q(1)-l_0)) - (( u*(q(1)+L*sin(q(2))))/q(1)) )/I ;




dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4;dqdt_5;dqdt_6];

end






%% 




function [value,isterminal,direction] = end_front_stance(~,q)  %between back leg stance and flight stance     %act on the back_stance_motion
	    % back leg liftoff event
       
		value = l_0 - q(1) ;
		isterminal =  1;
		direction =   -1;
end





end