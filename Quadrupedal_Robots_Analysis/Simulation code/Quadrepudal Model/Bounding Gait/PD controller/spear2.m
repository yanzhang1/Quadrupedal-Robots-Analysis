function q_3 = spear2(q_0,u_0)
%% 
%q_0 = [0.8397 4.1068];
%u_0 = [0.47619, 1.18988];
g = 9.8;
M = 4.6;
l_1 = 0.318;
l_2 = 0.318;
m_1 = 1.72;
m_2 = 0.47;
b = 0;
d_1 = 0.008;
d_2 = 0.144;
J_mh = 0.047;
J_mk = 0.027;
N_knee = 15;
N_hip = 60;
J_1 = 0.014;
J_2 = 0.005;

%Kp_hip = 54;
%Kd_hip = -2;
%Kp_knee = 250 ;
%Kd_knee = -2; 

A = J_mh*N_hip^2 + J_1 + m_1*d_1^2;
B = J_mk*N_knee^2 + J_2 + m_2*d_2^2;

%P = A + M*l_2^2  + M*l_1^2 + 2*M*l_1*l_2* cos(q(2));
%D = -M*l_2^2 - M*l_1*l_2*cos(q(2));
E = B + M*l_2^2;
%F = M*l_2^2 + M*l_1*l_2*cos(q(2));
G = M*g*l_2 + m_1*g*l_2 + m_2*g*l_2 - m_2*g*d_2;


options_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_stance); 
options_flight_1 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_1);
options_flight_2 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_2);

time= 100;
tt_end = 0;

t0 = 0;

theta1_td = u_0(1);
theta2_td = u_0(2);
q_3 = [100 100];
%% 

   %start at flight phase(terminate at touchdown event)
   [t1,q,~,~,events_flight_1] =ode45(@flight_motion, [t0,time] , ...
       [q_0(1) q_0(2) 0], options_flight_1);
   time1 = t1 + tt_end ;

   %q_1=[q(end,1) ,q(end,2),q(end,3)];
  % t1end = t1;
   %[From apex to touchdown]  Proceed to stance phase
   if events_flight_1 == 1
       q_1=[q(end,1) ,q(end,2),q(end,3)];
       time =time - t1(end);
       %get initial conditions from the termination event of flight phase
    theta1 = theta1_td;
    theta2 = theta2_td;
    thetadot1 = (cos(theta2_td - theta1_td)*q_1(3) + sin(theta2_td - theta1_td)*q_1(2))/(-0.318*sin(theta2_td));
    thetadot2 = (q_1(2) + thetadot1*(0.318*cos(theta2_td - theta1_td)+0.318*cos(theta1_td)))/(0.318*cos(theta2_td - theta1_td));
    q_td_0 = [theta1 theta2 thetadot1 thetadot2];
    [t2,q,~,~,events_stance] =ode45(@stance_motion, [0,time] ,...
        q_td_0, options_stance);
     time2 = t2 + time1(end);
        
    if events_stance == 1
    %plot(t2,q(:,4));
    q_2=[ q(end,1),q(end,2),q(end,3),q(end,4)];
    time = time - t2(end);
    %t2end  = t2;
       %get initial conditions from the termination event of stance phase
      yy = 0.318*cos(q_2(1)) + 0.318*cos(q_2(2)-q_2(1));
       xxdot = 0.318*cos(q_2(2)-q_2(1))*(q_2(4)-q_2(3)) - 0.318*cos(q_2(1))*q_2(3);
       yydot = -0.318*sin(q_2(1))*q_2(3) - 0.318*sin(q_2(2)-q_2(1))*(q_2(4)-q_2(3));
       qq_0 = [yy xxdot yydot];
    %[From take-off to apex]   Proceed to flight phase(terminate at apex height)
           
        [t3,q,~,~,~] =ode45(@flight_motion, [0,time] ,...
            qq_0, options_flight_2);
         q_3=[q(end,1),q(end,2)];
      time3 = t3 + time2(end);

         
     end
          
    
   end
norm1 = norm(q_3-q_0);
%% 

function dqdt=flight_motion(~,q)
%q=[y;xdot;ydot;theta_knee;theta_hip;thetadot_knee;thetadot_hip]
%dqdt=[ydot;xddot;yddot;thetadot_knee;thetadot_hip;thetaddot_knee;thetaddot_hip]

dqdt_1=q(3);
dqdt_2=0;
dqdt_3=-g;



dqdt=[dqdt_1;dqdt_2;dqdt_3];

end


function dqdt=stance_motion(~,q)
%q=[theta_1, theta_2 , thetadot_1 , thetadot_2]
%dqdt=[thetadot_1 , thetadot_2 , thetaddot_1  , thetaddot_2 ]
P = A + M*l_2^2  + M*l_1^2 + 2*M*l_1*l_2* cos(q(2));
D = -M*l_2^2 - M*l_1*l_2*cos(q(2));
F = M*l_2^2 + M*l_1*l_2*cos(q(2));

%tau_hip = Kp_hip*(q(1)-theta1_td) + Kd_hip*(q(3));
%tau_knee = Kp_knee*(q(2)-theta2_td) + Kd_knee*(q(4));
tau_hip = 0;
tau_knee = 0;

theta1_dot = (D/(P*E+D*F))* (   (E/D)*(tau_hip - M*l_1*l_2*q(4)^2*sin(q(2)) + 2*M*l_2*l_1*q(3)*q(4)*sin(q(2)) + (M*g*l_1 + m_1*g*l_1 - m_1*g*d_1)*sin(q(2)) - G*sin(q(2)-q(1))   )  + M*l_1*l_2*q(3)^2*sin(q(2)) -G*sin(q(2)-q(1)) - tau_knee + 2*b*q(4)^2               ) ;
dqdt_1 = q(3);
dqdt_2 = q(4);
dqdt_3 = theta1_dot ; 
dqdt_4 = (tau_hip - M*l_1*l_2*q(4)^2*sin(q(2)) + 2*M*l_2*l_1*q(3)*q(4)*sin(q(2)) + (M*g*l_1 + m_1*g*l_1 - m_1*g*d_1)*sin(q(2)) - G*sin(q(2)-q(1)) -P*theta1_dot  )*(1/D);



dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4];

end


%% 

function [value,isterminal,direction] = end_flight_1(~,q)
	%%Event 1 touchdown condition: 'y_touchdown' - 'r_0*cos(angle_of_attack)' =0;
    %%Event 2 fall down(point mass touch ground)
		value = q(1)-l_1*cos(theta1_td)-l_2*cos(theta2_td - theta1_td);
		isterminal =  1;
		direction =  -1;
end


function [value,isterminal,direction] = end_flight_2(~,q)
	%%Event 1 reach apex height
		value = q(3);
		isterminal =1;
		direction = -1;
end



function [value,isterminal,direction] = end_stance(~,q)
    %%Event 1 takeoff condition
    %%Event 2 point mass touch ground
    %%Event 3 point mass touch ground
    value = l_1*cos(theta1_td)+l_2*cos(theta2_td - theta1_td) - (  l_1*cos(q(1))+l_2*cos(q(2)-q(1))  );
    %value =sqrt(l_1^2 + l_2 ^2 - 2*l_1*l_2*cos(pi - theta2_td)) -sqrt(l_1^2 + l_2 ^2 - 2*l_1*l_2*cos(pi - q(2)));
		isterminal = 1;
		direction =-1;
end


end











