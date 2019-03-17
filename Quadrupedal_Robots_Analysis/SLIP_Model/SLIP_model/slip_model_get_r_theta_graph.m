function Data=slip_model_get_r_theta_graph(~,~)
%%
g=9.8;                         %gravity acceleration
k=20000;                       %spring constant
m=80;                          %mass
r_0=1;                         %rest length of leg
angle_of_attack=0.160*pi;%angle the model hold at beginning
% (x,y) is the position of point mass.
% 'r' is the length of leg and 'theta' is the angle formed by vertical line and the leg.

initial_x=0;                   %initial position along x
initial_y=1;                   %initial position along y
initial_xdot=6;                %initial horizontal speed
initial_ydot=0;                %initial vertical speed
Touchdown_theta=angle_of_attack; %initial angle
p_0=[initial_x   initial_y   initial_xdot   initial_ydot]; 

%define events to break or continue the phase
options_stance = odeset('Events', @end_stance); 
options_flight = odeset('Events', @end_flight);

%give time span
time=1000;

figure
hold on
%%
%%If first event(touchdown) of flight occurs,use ODE45 and update states and time,then discuss the events of stance.If second event(fall down) of flight occurs, break and end. 
%%If first event(liftoff) of stance occurs,use ODE45 and update states and time.If second event of stance occurs,break and end.
for i=1:1:1000
	% Start with flight stance
    %use ode45 to get 'p'
    %If you specify an events function, you can call the ODE solver with extra output arguments as
    %[t,y,te,ye,ie] = odeXY(odefun,tspan,y0,options)
    %'ie' are indices into the vector returned by the events function. The values indicate which event the solver detected.
    %'events_flight' refers to 'ie' 
    [t,p,~,~,events_flight] =ode45(@flight_motion, [0,time] ,  p_0 , options_flight);
    %store data
	p=[p(:,1) , p(:,2) , p(:,3:4)]; %p(:,1) means first column
    time=time-t(end);
    if events_flight == 2
        break
    end
	% Proceed to stance motion
    if events_flight==1
        x_toe=p(end,1)+ r_0*sin(Touchdown_theta);%p(end,:) means last row
       %Give initial conditions to Stance motion, mainly by applying jacobian matrix to transfer the end state value of 'x_dot' and 'y_dot' expressed in (x,y) to the 'r_dot_touchdown' and 'theta_dot_touchdown'
      rdot_touchdown=-sin(Touchdown_theta)*p(end,3)+cos(Touchdown_theta)*p(end,4);
      thetadot_touchdown=(-cos(Touchdown_theta)*p(end,3)-sin(Touchdown_theta)*p(end,4))/r_0;
      %use ode45 to get 'q'
        [t,q,~,~,events_stance] = ode45(@stance_motion, [0,time], [r_0   Touchdown_theta   rdot_touchdown   thetadot_touchdown] , options_stance);
 
       if events_stance == 4
          break
       end
	      Data(i).q=[q(:,1) ,q(:,2), q(:,3:4)];
		  time=time-t(end); 
          
          p_stance=[x_toe-q(end,1)*sin(q(end,2)),...
                    q(end,1)*cos(q(end,2)),...
                    -q(end,3)*sin(q(end,2))-q(end,1)*q(end,4)*cos(q(end,2)),...
                    q(end,3)*cos(q(end,2))-q(end,1)*q(end,4)*sin(q(end,2))]; 
          p_0=p_stance; 
         
          Touchdown_theta=-q(end,2);
    else 
        break
    end
    plot(Data(i).q(:,2),Data(i).q(:,1));
    xlabel('theta(Radians)');
    ylabel('r(m)');
    title('angle of attack is 0.160*pi');
    legend('First st.ph.','Second st.ph.','Third st.ph.','Fourth st.ph.','Fifth st.ph.','Sixth st.ph.','Seventh st.ph.','...');
end



function dpdt=flight_motion(~,p)
%p=[x;y;xdot;ydot]
%dpdt=[xdot;ydot;xddot;yddot]
dpdt_1=p(3);
dpdt_2=p(4);
dpdt_3=0;
dpdt_4=-g;

dpdt=[dpdt_1; dpdt_2; dpdt_3; dpdt_4];
end
       
function dqdt=stance_motion(~,q)
%q=[r;theta;rdot;thetadot]
%dqdt=[rdot;thetadot;rddot;thetaddot]
dqdt_1=q(3);
dqdt_2=q(4);
dqdt_3=-(k/m)*(q(1)-r_0)-g*cos(q(2))+q(1)*q(4)^2;
dqdt_4=g/q(1)*sin(q(2))-2/q(1)*q(3)*q(4);

dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4];

end
%%
%%Explain following functions:
%%1.value(i) is a mathematical expression describing the ith event. An event occurs when value(i) is equal to zero. 
%%2.isterminal(i) = 1 if the integration is to terminate when the ith event occurs. Otherwise, it is 0.
%%3.direction(i) = 0 if all zeros are to be located (the default). A value of +1 locates only zeros where the event function is increasing, and -1 locates only zeros where the event function is decreasing. Specify direction = [] to use the default value of 0 for all events.

%%flight phase end(including several events,like touchdown,fall down)
function [value,isterminal,direction] = end_flight(~,p)
	%%Event 1 touchdown condition: 'y_touchdown' - 'r_0*cos(angle_of_attack)' =0;
    %%Event 2 fall down(point mass touch ground)
		value = [ p(2) - r_0*cos(Touchdown_theta),p(2)];
		isterminal = [ 1, 1];
		direction = [-1,-1];
end

%%Stance phase end(including several events, like takeoff , falldown , )
function [value,isterminal,direction] = end_stance(~,q)
    %%Event 1(a) takeoff condition_1 : 'r_liftoff' - 'r_0'=0;
    %%Event 1(b) takeoff condition_2 : 'rdot_touchdown' + rdot_takeoff=0;
    %%Event 3 point mass touch ground
		value = [p(2) - r_0*cos(Touchdown_theta),q(3)+rdot_touchdown,q(4)-thetadot_touchdown,q(2)-pi/2];
		isterminal = [1,1,1,1];
		direction = [1,1,1,0];
end
%%get whole graph 

end
