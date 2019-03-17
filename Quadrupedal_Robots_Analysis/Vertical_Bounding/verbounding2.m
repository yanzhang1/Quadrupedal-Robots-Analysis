function data=verbounding2(~)
%% 

g=9.8;                         %gravity acceleration
k=3520;                        %spring constant
m=19.32;                       %mass
l_0=0.3;                       %rest length of leg
b = 45;                        %damping constant

initial_y= 0.6 ;                 %initial position along y
initial_ydot= 0 ;                %initial vertical speed

q_0=[initial_y   initial_ydot]; 


options_flight = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight); 
options_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_stance);

time= 600;


figure
hold on
%%
for i=1:10

   [t,q,~,~,events_flight] =ode45(@flight_motion, (0:0.001:time) , ...
       q_0, options_flight);
   time =time - t(end);
   data(2*i-1).q = [q(:,1), q(:,2)];
   q_1=[q(end,1) ,q(end,2)];
    time =time - t(end);
  
   if events_flight == 1
    [t,q,~,~,events_stance] =ode45(@stance_motion, (0:0.001:time) ,...
       q_1, options_stance);
   data(2*i).q = [q(:,1),q(:,2)];
   if events_stance == 2 
       break 
   end
   if events_stance == 1
    q_0=[q(end,1) ,q(end,2)];
    time = time - t(end);

   end
    
   end
    
   plot(data(i).q(:,1),data(i).q(:,2));
    xlabel('vertical position of the torso(m)');
    ylabel('vertical velocity of the torso(m/s)');
    title('Leg length is 0.3 m, Initial release height is 0.6 m' );
    txt_1 = {'Flight phase'};
    text(0.4,0,txt_1);
    txt_2 = {'Stance phase'};
    text(0.15,0,txt_2);
end

%%
function dpdt=flight_motion(~,q)
%p=[y;ydot]
%dpdt=[ydot; yddot]
dqdt_1= q(2) ;
dqdt_2= -g ;

dpdt=[dqdt_1; dqdt_2];
end
       
function dqdt=stance_motion(~,q)
%q=[y;ydot]
%dqdt=[ydot; yddot]
dqdt_1=q(2);
dqdt_2=((2*k)/m)*(l_0 - q(1)) - ((2*b)/m)*q(2) - g;


dqdt=[dqdt_1;dqdt_2];

end


%% 

function [value,isterminal,direction] = end_flight(~,q)
		value =  q(1) - l_0 ;
		isterminal =  1;
		direction = -1;
end




function [value,isterminal,direction] = end_stance(~,q)   
		value = [q(1)- l_0  ];
		isterminal = [1 ];
		direction =[ 1 ];
end


end