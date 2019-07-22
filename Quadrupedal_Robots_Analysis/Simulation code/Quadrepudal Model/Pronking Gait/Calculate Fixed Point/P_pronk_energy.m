function data=P_pronk_energy(q_0,u_0)
%% 
g=9.81;                         %gravity acceleration 
k=3520;                         %spring constant
m=20.865;                        %torso mass
l_0=0.323;                      %rest length of leg
I = 1.3;                          %damping constant


%angle the model hold at beginning
%0.146225*pi
  %initial conditions [y theta xdot ydot]

options_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_stance); 
options_flight_1 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_1);
options_flight_2 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_2);

time= 60;
figure
hold on
%% 
  for i = 1:3
   %start at flight phase(terminate at touchdown event)
   [t1,q,~,~,events_flight_1] =ode45(@flight_motion, [0,time] , ...
       [q_0(1) q_0(2) 0], options_flight_1);
   time =time - t1(end);
   q_1=[q(end,1) ,q(end,2),q(end,3)];
   data(3*i-2).time = t1;
   data(3*i-2).q = [q(:,1),q(:,2),q(:,3)];
   data(3*i-2).energy = (1/2)*m.*(q(:,2).^2+q(:,3).^2) + m*g.*q(:,1);
 %  data(3*i-2).potential =  zeros(length(time),1) ;
   
  % fprintf('%f \n',(1/2)*m*(q_1(2)^2+q_1(3)^2) + m*g*q_1(1));
   %[From apex to touchdown]  Proceed to stance phase
   if events_flight_1 == 1
       %get initial conditions from the termination event of flight phase
       rdot_touchdown=-sin(u_0)*q_1(2)+cos(u_0)*q_1(3);
       thetadot_touchdown=(-cos(u_0)*q_1(2)-sin(u_0)*q_1(3))/l_0;
       
    [t2,q,~,~,events_stance] =ode45(@stance_motion, [0,time] ,...
        [l_0  u_0  rdot_touchdown  thetadot_touchdown], options_stance);
    q_2=[ q(end,1),q(end,2),q(end,3),q(end,4)];
    time = time - t2(end);
    
    y_1 = q(:,1).*cos(q(:,2)) ;
    ydot = q(:,3).*cos(q(:,2)) - q(:,1).*q(:,4).*sin(q(:,2));
    xdot = -q(:,3).*sin(q(:,2)) - q(:,1).*q(:,4).*cos(q(:,2)); 
    data(3*i-1).time = t2 + t1(end);
    data(3*i-1).q = [q(:,1),q(:,2),q(:,3),q(:,4)];
    data(3*i-1).energy =  m*g.*y_1 +(1/2)*m*(xdot.^2 + ydot.^2) + k*(l_0 - q(:,1)).^2 ;
  %  data(3*i-1).potential = k*(l_0 - q(:,1)).^2;
     
       %get initial conditions from the termination event of stance phase
       
       y_liftoff = l_0*cos(q_2(2));
       xdot_liftoff = -sin(q_2(2))*q_2(3)-l_0*cos(q_2(2))*q_2(4);
       ydot_liftoff =cos(q_2(2))*q_2(3) - l_0*sin(q_2(2))*q_2(4);
    
    %[From take-off to apex]   Proceed to flight phase(terminate at apex height)
           if events_stance == 1
        [t3,q,~,~,~] =ode45(@flight_motion, [0,time] ,...
            [y_liftoff    xdot_liftoff    ydot_liftoff ], options_flight_2);
         q_0=[ q(end,1),q(end,2)];
          data(3*i).time = t3 +t2(end) + t1(end);
         data(3*i).q = [q(:,1),q(:,2),q(:,3)];
        data(3*i).energy = (1/2)*m.*(q(:,2).^2+q(:,3).^2) + m*g.*q(:,1);
    %    data(3*i-1).potential =  zeros(length(time),1)  ;
         time = time - t3(end);
           end
   end
   
   plot(data(i).time,data(i).energy);
  % plot( data(3*i-1).time , data(3*i-1).potential);
   xlabel('Time (s)');
 %  ylabel('Mechanical Energy (J)');
   ylabel('Potential Energy store in Spring legs (J)');
 %  title('Pronking Gait from one Apex to the next Apex');
    title('Potential Energy versus Time in stance phase');
 %  legend('Flight phase','Stance phase','Flight phase');
  end
%% 

function dqdt=flight_motion(~,q)
%q=[y;xdot;ydot]
%dqdt=[ydot;xddot;yddot]
dqdt_1=q(3);
dqdt_2=0;
dqdt_3=-g;

dqdt=[dqdt_1;dqdt_2;dqdt_3];

end


function dqdt=stance_motion(~,q)
%q=[l ; phi ; ldot ; phidot]
%dqdt=[ldot ; phidot ; lddot  ; phiddot]

dqdt_1=q(3);
dqdt_2=q(4);
dqdt_3=(2*(k/m))*(l_0-q(1))-g*cos(q(2))+q(1)*q(4)^2 ;%  - (2*(b/m))*q(3);
dqdt_4=(g*sin(q(2)))/q(1)-(2*q(3)*q(4))/q(1);


dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4];

end


%% 

function [value,isterminal,direction] = end_flight_1(~,q)
	%%Event 1 touchdown condition: 'y_touchdown' - 'r_0*cos(angle_of_attack)' =0;
    %%Event 2 fall down(point mass touch ground)
		value = [ q(1)-cos(u_0)*l_0,q(1),q(2)];
		isterminal = [ 1,1,1];
		direction =  [-1,-1,-1];
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
		value = [l_0-q(1),q(2)-pi/2,q(3),q(4)];
		isterminal = [1,1,1,1];
		direction = [-1,-1,-1,1];
end


end