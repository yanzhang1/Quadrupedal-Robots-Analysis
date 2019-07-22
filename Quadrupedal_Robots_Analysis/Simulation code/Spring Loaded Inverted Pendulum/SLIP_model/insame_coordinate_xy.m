function Data=insame_coordinate_xy(~,~)
%%
g=9.8;                         %gravity acceleration
k=20000;                       %spring constant
m=80;                          %mass
r_0=1;                         %rest length of leg
angle_of_attack=0.135*pi;%angle the model hold at beginning
% (x,y) is the position of point mass.
% 'r' is the length of leg and 'theta' is the angle formed by vertical line and the leg.

initial_x=0;                   %initial position along x
initial_y=0.999255;                   %initial position along y
initial_xdot=5.177697;                %initial horizontal speed
initial_ydot=0;                %initial vertical speed


p_0=[initial_x   initial_y   initial_xdot   initial_ydot]; 

%define events to break or continue the phase
options_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_stance); 
options_flight = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight);

%give time span
time=1000;
figure
hold on
%%
%%If first event(touchdown) of flight occurs,use ODE45 and update states and time,then discuss the events of stance.If second event(fall down) of flight occurs, break and end. 
%%If first event(liftoff) of stance occurs,use ODE45 and update states and time.If second event of stance occurs,break and end.
for i=1:500
    [t,p,~,~,events_flight] =ode45(@flight_motion, [0,time] ,  p_0 , options_flight);
	Data(2*i-1).p=[p(:,1),p(:,2),p(:,3), p(:,4)]; %p(:,1) means first column
    time=time-t(end);
    p_0=p(end,:);
    x_toe=p(end,1)+ r_0*sin(angle_of_attack);
    if events_flight == 2
        break
    end
    if events_flight==1
      
	   [t,p,~,~,events_stance] = ode45(@stance_motion, [0,time], p_0 , options_stance);
    if events_stance == 2
        break
    end
	      Data(2*i).p=[p(:,1),p(:,2),p(:,3), p(:,4)];
		  time=time-t(end); 
          p_0=p(end,:);
    end    
    plot(Data(i).p(:,1),Data(i).p(:,2));
    xlabel('x position of point mass(m)');
    ylabel('y position of point mass(m)');
    title('angle of attack is 0.160*pi' );
    %legend('flight phase','stance phase','flight phase','stance phase','flight phase','...');
end

%%
function dpdt=flight_motion(~,p)
%p=[x;y;xdot;ydot]
%dpdt=[xdot;ydot;xddot;yddot]
dpdt_1=p(3);
dpdt_2=p(4);
dpdt_3=0;
dpdt_4=-g;

dpdt=[dpdt_1; dpdt_2; dpdt_3; dpdt_4];
end
       
function dpdt=stance_motion(~,p)
%p=[x;y;xdot;ydot]
%dpdt=[xdot;ydot;xddot;yddot]
x_toe=p_0(1)+ r_0*sin(angle_of_attack);
r=sqrt((x_toe-p(1))^2+p(2)^2);
theta=asin((x_toe-p(1))/r);
dpdt_1=p(3);
dpdt_2=p(4);
dpdt_3=(k/m)*(r-r_0)*sin(theta);
dpdt_4=-g-(k/m)*(r-r_0)*cos(theta);

dpdt=[dpdt_1;dpdt_2;dpdt_3;dpdt_4];

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
		value = [p(2)-r_0*cos(angle_of_attack),p(2)];
		isterminal = [ 1, 1];
		direction =  [-1,-1];
end

%%Stance phase end(including several events, like takeoff , falldown , )
function [value,isterminal,direction] = end_stance(~,p)
    %%Event 1(a) takeoff condition_1 : 'r_liftoff' - 'r_0'=0;
    %%Event 1(b) takeoff condition_2 : 'rdot_touchdown' + rdot_takeoff=0;
    %%Event 3 point mass touch ground
	%                                                                            Wrong!!!! 
    value = [p(2)-r_0*cos(angle_of_attack),...
           p(2)];
		isterminal = [1,1];
		direction = [1,-1];
end

end