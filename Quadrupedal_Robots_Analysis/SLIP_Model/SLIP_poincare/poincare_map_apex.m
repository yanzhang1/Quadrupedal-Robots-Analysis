function data=poincare_map_apex(~)
%% 

g=9.81;                         %gravity acceleration 
k=20000;                       %spring constant
m=80;                          %mass
r_0=1;                         %rest length of leg
angle_of_attack=0.145*pi;%angle the model hold at beginning
%0.146225*pi

q_0=[1  angle_of_attack  6.5  0];  %initial conditions [y theta xdot ydot]

options_stance = odeset('RelTol',1e-5,'AbsTol',1e-4,'Events', @end_stance); 
options_flight_1 = odeset('RelTol',1e-5,'AbsTol',1e-4,'Events', @end_flight_1);
options_flight_2 = odeset('RelTol',1e-5,'AbsTol',1e-4,'Events', @end_flight_2);

time= 6000;
figure 
hold on 
%% 

for i=1:1000
   data(i).apex_state = [q_0(1)   angle_of_attack   q_0(3)    q_0(4)];
   %start at flight phase(terminate at touchdown event)
   [t,q,~,~,events_flight_1] =ode45(@flight_motion, [0,time] , ...
       [q_0(1)   angle_of_attack   q_0(3)    q_0(4)] , options_flight_1);
   time =time - t(end);
   q_0=[q(end,1) ,q(end,2),q(end,3),q(end,4)];
   %[From apex to touchdown]  Proceed to stance phase
   if events_flight_1 == 1
       %get initial conditions from the termination event of flight phase
       rdot_touchdown=-sin(angle_of_attack)*q_0(3)+cos(angle_of_attack)*q_0(4);
       thetadot_touchdown=(-cos(angle_of_attack)*q_0(3)-sin(angle_of_attack)*q_0(4))/r_0;
       
    [t,q,~,~,events_stance] =ode45(@stance_motion, [0,time] ,...
        [r_0  angle_of_attack  rdot_touchdown  thetadot_touchdown], options_stance);
    q_0=[ q(end,1),q(end,2),q(end,3),q(end,4)];
    time = time - t(end);
       %get initial conditions from the termination event of stance phase
       y_liftoff = r_0*cos(pi+q_0(2));
       theta_liftoff = pi+q_0(2);
       xdot_liftoff = -sin(pi+q_0(2))*q_0(3)-r_0*cos(pi+q_0(2))*q_0(4);
       ydot_liftoff = cos(pi+q_0(2))*q_0(3) - r_0*sin(pi+q_0(2))*q_0(4);
    
    %[From take-off to apex]   Proceed to flight phase(terminate at apex height)
           if events_stance == 1
        [t,q,~,~,~] =ode45(@flight_motion, [0,time] ,...
            [y_liftoff   theta_liftoff  xdot_liftoff    ydot_liftoff ], options_flight_2);
          q_0=[ q(end,1) ,q(end,2),q(end,3),q(end,4)];
           time = time - t(end);
          else 
              break
          end
   else
       break
   end
   plot(i,data(i).apex_state(:,1),'+');
    xlabel('i^{th} iteration');
    ylabel('y(m)_apex height');
    title('ICs are [y=1m , theta=0.146225*pi , xdot=6m/s , ydot=0]');
    
end
%% 

function dqdt=flight_motion(~,q)
%q=[y;theta;xdot;ydot]
%dqdt=[ydot;thetadot;xddot;yddot]
dqdt_1=q(4);
dqdt_2=0;
dqdt_3=0;
dqdt_4=-g;


dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4];

end


function dqdt=stance_motion(~,q)
%q=[r ; theta ; rdot ; thetadot]
%dqdt=[rdot ; thetadot ; rddot  ; thetaddot]

dqdt_1=q(3);
dqdt_2=q(4);
dqdt_3=-(k/m)*(q(1)-r_0)-g*cos(q(2))+q(1)*q(4)^2;
dqdt_4=(g*sin(q(2)))/q(1)-(2*q(3)*q(4))/q(1);


dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4];

end


%% 

function [value,isterminal,direction] = end_flight_1(~,q)
	%%Event 1 touchdown condition: 'y_touchdown' - 'r_0*cos(angle_of_attack)' =0;
    %%Event 2 fall down(point mass touch ground)
		value = [ q(1)-cos(angle_of_attack)*r_0,q(1),q(3)];
		isterminal = [ 1,1,1];
		direction =  [-1,-1,-1];
end


function [value,isterminal,direction] = end_flight_2(~,q)
	%%Event 1 reach apex height
		value = q(4);
		isterminal =1;
		direction = -1;
end



function [value,isterminal,direction] = end_stance(~,q)
    %%Event 1 takeoff condition
    %%Event 2 point mass touch ground
    %%Event 3 point mass touch ground
		value = [r_0-q(1),q(2)-pi/2,q(3),q(4)];
		isterminal = [1,1,1,1];
		direction = [-1,-1,-1,1];
end


end

