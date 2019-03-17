function q_7 = allmo(q_0,la)
%% 
g=9.8;                         %gravity acceleration 
k=3520;                         %spring constant
m=20.865;                        %torso mass
l_0=0.323;                       %rest length of leg
L = 0.276;                       % half length of torso
I = 1.3;
%u_0 = [phi_bTD,phi_fTD]
%u_0(1) = [phi_bTD] back leg touchdown angle
%u_0(2) = [phi_fTD] front leg touchdown angle
%q_0 = [0.325814	 0.0000	 0.673839	 6.117373	];
%la = [0.226892803  ,0.191986218 ];
%q_0 = [0.3279,  -0.0, 0.6771, 5.9781 ];
%%%%%%%%%%%%%%%%%%%q_0 = ;
%la = [0.2269 ,0.1920];[ 0.32988230089786857446298995455436, -0.00000000081439788110995878437014967177058, 0.68086293864421565391609192374744, 5.8266263375095483922905259532854]
options_flight_1 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_1);
options_back_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_back_stance);
options_flight_2 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_2);
options_front_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_front_stance);
options_flight_3 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_3);
tt_end = 0;
time= 10;
t0 = 0;
x_0 = 20;
ydot_0 = 0;
q_7 = [-100 -100 -100 -100];
%la = u_0;
%i = 1;
%fprintf('The total energy at first apex height is : %f \n',(1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1));
%% 
%%%%%%%%%%%%%%%%%%%%%for i = 1:20
  %%
   [t1,q,~,~,events_flight_1] =ode45(@flight_motion, [t0,time] , [x_0 , q_0(1), q_0(2) ,q_0(3),  ydot_0 , q_0(4)], options_flight_1);
  %%
   q = [q(:,1) ,q(:,2) ,q(:,3) ,q(:,4) ,q(:,5) ,q(:,6) ];
   if exist('q','var')
      feas1 = 1; 
   else
       feas1 = 0;
   end
   time1 = t1 + tt_end ;
   %qq(6*i-5).time = time1;
   %qq(6*i-5).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
   
   if events_flight_1 == 1 
       
        
   q_1=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
  
   
   time =time - t1(end);
   x_btoe = q_1(1) - L*cos(q_1(3)) + l_0*sin(la(1));
   %%   
   mm = x_btoe + L*cos(q_1(3)) - q_1(1);
   nn = L*sin(q_1(3)) - q_1(2);
   ll_0 = sqrt(mm^2+nn^2) ;
   la_0 = la(1) - q_1(3);
   theta_0 = q_1(3);

   lb_dot =   (-mm*(L*q_1(6)*sin(q_1(3)) + q_1(4))  + nn*(L*q_1(6)*cos(q_1(3)) -q_1(5)))/ll_0;
   phib_dot = - (cos(la(1)) )^2 * (      (L*q_1(6)*sin(q_1(3))+ q_1(4))/(-L*sin(q_1(3)) + q_1(2))  + (mm*(-L*q_1(6)*cos(q_1(3)) +q_1(5)))/(-L*sin(q_1(3)) + q_1(2))^2   )  - q_1(6);
   thetadot_0 = q_1(6);
   
   backstance_0 = [ll_0 , la_0 , theta_0 , lb_dot , phib_dot, thetadot_0];
   %%
    [t2,q,~,~,events_back_stance] =ode45(@back_stance_motion, [t0,time] , backstance_0, options_back_stance);
   %%
   q = [q(:,1) ,q(:,2) ,q(:,3) ,q(:,4) ,q(:,5) ,q(:,6) ];
   if exist('q','var') 
      feas2 = 1;
   else
       feas2 = 0;
   end
     time2 = t2 + time1(end);
   %  qq(6*i-4).time = time2;
  %   qq(6*i-4).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
  
    if events_back_stance == 1 %(back leg liftoff)    % proceed back leg stance phase to flight phase
    
        q_2=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
            %[l , phi , theta , ldot, phidot , thetadot]
        time = time - t2(end);
        %% 
        x_0 = x_btoe + L*cos(q_2(3)) - q_2(1)*sin(q_2(2)+q_2(3))  ;
        y_0 = L*sin(q_2(3)) + q_2(1)*cos(q_2(2)+q_2(3)) ;
        theta2_0 = q_2(3); 
        xdot_0 = - q_2(4)*sin(q_2(2)+q_2(3)) - q_2(1)*cos(q_2(2)+q_2(3))*(q_2(5)+q_2(6)) - L*q_2(6)*sin(q_2(3));
        ydot_0 =   q_2(4)*cos(q_2(2)+q_2(3)) - q_2(1)*sin(q_2(2)+q_2(3))*(q_2(5)+q_2(6)) + L*q_2(6)*cos(q_2(3));
        thetadot2_0 = q_2(6) ;
        
        
        %%

        [t3,q,~,~,events_flight_2] =ode45(@flight_motion, [t0,time] ,[x_0 , y_0 , theta2_0 , xdot_0 , ydot_0 , thetadot2_0] , options_flight_3);
       time3 = t3 +time2(end);
       
       q = [q(:,1) ,q(:,2) ,q(:,3) ,q(:,4) ,q(:,5) ,q(:,6) ];
   if exist('q','var')
      feas3 = 1; 
   else
       feas3 = 0;
   end
    %   qq(6*i-3).time = time3;
    %   qq(6*i-3).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
        if events_flight_2 == 1 %( reach apex height)   % proceed flight to apex
         q_3=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
         
         %  =[x       ,y,      ,theta   ,xdot,   ,ydot    ,thetadot]
         time =time - t3(end);
         
        [t4,q,~,~,events_flight_3] =ode45(@flight_motion, [t0,time] ,[q_3(1),q_3(2),q_3(3),q_3(4),q_3(5),q_3(6)] , options_flight_2);
        time4 = t4 +time3(end)  ;
        q = [q(:,1) ,q(:,2) ,q(:,3) ,q(:,4) ,q(:,5) ,q(:,6) ];
   if exist('q','var')
      feas4 = 1; 
   else
       feas4 = 0;
   end
    %    qq(6*i-2).time = time4; 
    %    qq(6*i-2).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
    %    
        
        
        
        if events_flight_3 == 1 %(end flight, front leg touch down)
             q_4=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
     
             time = time - t4(end);
      
            x_ftoe= l_0*sin(la(2)) + q_4(1) + L*cos(q_4(3));
             
            %%
             mm2 = x_ftoe - L*cos(q_4(3)) - q_4(1);
             nn2 = L*sin(q_4(3)) + q_4(2);
             llf_0 = sqrt(mm2^2+nn2^2) ;
             laf_0 = la(2) - q_4(3);
             theta3_0 = q_4(3);
             
             lf_dot =   (mm2*(L*q_4(6)*sin(q_4(3)) - q_4(4))  + nn2*(L*q_4(6)*cos(q_4(3)) +q_4(5)))/llf_0;
             phif_dot = - (cos(la(2)) )^2 * (      (-L*q_4(6)*sin(q_4(3))+ q_4(4))/nn2  + (mm2*(L*q_4(6)*cos(q_4(3)) +q_4(5)))/nn2^2   )  - q_4(6);
             thetadot3_0 = q_4(6);
            frontstance_0 = [llf_0 , laf_0 , theta3_0 , lf_dot , phif_dot, thetadot3_0];
            %%
              [t5,q,~,~,events_front_stance] =ode45(@front_stance_motion, [t0,time] , frontstance_0 , options_front_stance);
              time5 = t5 + time4(end) ;
           %   qq(6*i-1).time = time5 ;
           %   qq(6*i-1).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
  
               if events_front_stance == 1 %(front leg liftoff)   % proceed front leg stance phase to flight phase
              q_5=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];       
               
              time =time - t5(end);
              %%
              
               x4_0 = x_ftoe - L*cos(q_5(3)) - q_5(1)*sin(q_5(2)+q_5(3))  ;
               y4_0 = -  L*sin(q_5(3)) + q_5(1)*cos(q_5(2)+q_5(3)) ;
               theta4_0 = q_5(3); 
               xdot4_0 = - q_5(4)*sin(q_5(2)+q_5(3)) - q_5(1)*cos(q_5(2)+q_5(3))*(q_5(5)+q_5(6)) + L*q_5(6)*sin(q_5(3));
               ydot4_0 =   q_5(4)*cos(q_5(2)+q_5(3)) - q_5(1)*sin(q_5(2)+q_5(3))*(q_5(5)+q_5(6)) - L*q_5(6)*cos(q_5(3));
               thetadot4_0 = q_5(6);
           
              %%
            
                  [t6,q,~,~,~] =ode45(@flight_motion, [t0,time] , [x4_0 , y4_0 , theta4_0 , xdot4_0 , ydot4_0 ,thetadot4_0] , options_flight_3);
                  time6 = t6 + time5(end) ;
                %  qq(6*i).time = time6 ;
               %   qq(6*i).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
                   q_6 = [q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
                   q_7=[q_6(2),q_6(3),q_6(4),q_6(6)];
                   
                   
                 %  bb = norm(q_7);
               %   fprintf('The total energy at next apex height is : %f \n',(1/2)*m*(q(4)^2 + q(5)^2) + (1/2)*I*q(6)^2 + m*g*q(2));
        %      time =time - t6(end);
         %      tt_end = time6(end);
         %      x_0 = q_6(1);
         %%      ydot_0 = q_6(5);
          %     q_0 = q_7;
         %      t0 = t0 + 0.0000001;
          %     time =time - t6(end);
          %     tt_end = time6(end);
          %     q_0 = q_7;
               %   fprintf('%f \n',((1/2)*m*(q(4)^2 + q(5)^2) + (1/2)*I*q(6)^2 + m*g*q(2))-((1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1)));
               end
       
        end
  
        end
  
    end
   end
   
%%%%%%%%%%%%%%%%end
 %  delta_bs = q_2(3)+q_2(2) -backstance_0(2)- backstance_0(3);
  % state_theta_bs = qq(2).state(:,3);
 %  state_legangle_bs = qq(2).state(:,2);
 %  s =( state_theta_bs + state_legangle_bs -backstance_0(2)- backstance_0(3))/delta_bs;
 %  m=6;
 %  n = length(state_theta_bs);
 %  A = zeros(n,m);
 %   for i = 1:n 
   %     for j = 1:m
  %        A(i,j) = ( factorial(m-1)/(factorial(j-1)*factorial(m - j))) * s(i)^(j-1) *(1-s(i))^(m-j) ; 
  %      end
  %  end
    
   % alpha_bs = zeros(6,1);
  %  alpha_bs = A\state_theta_bs;
   
   
    
    
    
      
 %  delta_fs = q_5(3)+q_5(2) - frontstance_0(3)-frontstance_0(2);
 %  state_theta_fs = qq(2).state(:,3);
  % state_leg_fs = qq(2).state(:,2);
  % s =( state_theta_fs + state_leg_fs  - frontstance_0(3)-frontstance_0(2))/delta_fs;
 %  m=6;
 %  n = length(state_theta_fs);
 %  A = zeros(n,m);
  %  for i = 1:n 
  %      for j = 1:m
  %        A(i,j) = ( factorial(m-1)/(factorial(j-1)*factorial(m - j))) * s(i)^(j-1) *(1-s(i))^(m-j) ; 
  %      end
  %  end
    
   % alpha_bs = zeros(6,1);
 %   alpha_fs = A\state_theta_fs;
   
   
  %     figure
  % hold on
 %for i =1:6
   
 %figure(1),plot(qq(i).time,qq(i).state(:,3));
 %end
 %  plot(qq(i).time,qq(i).state(:,3));
%end
%% 

function dqdt=flight_motion(~,q)
%q=[x;y;theta;xdot;ydot;thetadot]
%dqdt=[xdot;ydot;thetadot;xddot;yddot;thetaddot]
dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4= 0 ;
dqdt_5= -g;
dqdt_6= 0 ;


dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4;dqdt_5;dqdt_6];

end


function dqdt=back_stance_motion(~,q)
%q=[l,phi,theta,ldot,phidot,thetadot,(desired),l,phi,theta,ldot,phidot,thetadot]
%dqdt=[]
%Before this motion, indicate what is 'x_btoe'
%[alpha,delta] = provide_desired_theta_forbs; 
%s = (q(3) - theta_0) /delta;
%theta_des = alpha(1)*( factorial(5)/(factorial(0)*factorial(5))) * s^0 * (1-s)^5 + alpha(2)*( factorial(5)/(factorial(1)*factorial(4))) * s^1 * (1-s)^4 + ...
 %            alpha(3)*( factorial(5)/(factorial(2)*factorial(3))) * s^2 * (1-s)^3 + alpha(4)*( factorial(5)/(factorial(3)*factorial(2))) * s^3 * (1-s)^2 + ...
  %           alpha(5)*( factorial(5)/(factorial(4)*factorial(1))) * s^4 * (1-s)^1 + alpha(6)*( factorial(5)/(factorial(5)*factorial(0))) * s^5 * (1-s)^0 ; 

%u = -((I*q(1))/(q(1)-L*sin(q(2))))*(-Kp* (q(3)-theta_des) - Kd * q(6) - ( k*L*cos(q(2))*(q(1)-l_0) )/I );
u=0;
dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4=-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u)/q(1)    )/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
dqdt_5=-(k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))  + ( ((q(1)-L*sin(q(2)))^2)/(I*q(1)^2)  + 1/(m*(q(1)^2)))*u - (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
dqdt_6= ( +(k*L*cos(q(2))*(q(1)-l_0)) - (( u*(q(1)-L*sin(q(2))))/q(1)) )/I ;



%dqdt_1=q(4);
%dqdt_2=q(5);
%dqdt_3=q(6);
%dqdt_4=-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0))/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
%dqdt_5=-(k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))  - (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
%dqdt_6= ( +(k*L*cos(q(2))*(q(1)-l_0)))/I ;

dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4;dqdt_5;dqdt_6];

end




function dqdt=front_stance_motion(~,q)
%q=[l,phi,theta,ldot,phidot,thetadot,(desired),l,phi,theta,ldot,phidot,thetadot]
%dqdt=[]
%Before this motion, indicate what is 'x_btoe'
%[alpha,delta] = provide_desired_theta_forfs; 
%s = (q(3) - theta3_0) /delta;

%theta_des = alpha(1)*( factorial(5)/(factorial(0)*factorial(5))) * s^0 * (1-s)^5 + alpha(2)*( factorial(5)/(factorial(1)*factorial(4))) * s^1 * (1-s)^4 + ...
   %          alpha(3)*( factorial(5)/(factorial(2)*factorial(3))) * s^2 * (1-s)^3 + alpha(4)*( factorial(5)/(factorial(3)*factorial(2))) * s^3 * (1-s)^2 + ...
   %          alpha(5)*( factorial(5)/(factorial(4)*factorial(1))) * s^4 * (1-s)^1 + alpha(6)*( factorial(5)/(factorial(5)*factorial(0))) * s^5 * (1-s)^0 ; 

%theta_des = alpha(1) + alpha(2)*s + alpha(3)*s^2 +alpha(4)*s^3 + alpha(5)*s^4 + alpha(6)*s^5;

u =0;

dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4=( - k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u)/q(1)    )/I  + L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
dqdt_5=(k*L*cos(q(2))*(q(1)-l_0)*(q(1)+L*sin(q(2))))/(I*q(1))  + ( ((q(1)+L*sin(q(2)))^2)/(I*q(1)^2)  + 1/(m*(q(1)^2)))*u  + (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
dqdt_6= ( -(k*L*cos(q(2))*(q(1)-l_0)) - (( u*(q(1)+L*sin(q(2))))/q(1)) )/I ;



%dqdt_1=q(4);
%dqdt_2=q(5);
%dqdt_3=q(6);
%dqdt_4=( - k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0)     )/I  + L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
%dqdt_5=(k*L*cos(q(2))*(q(1)-l_0)*(q(1)+L*sin(q(2))))/(I*q(1))   + (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
%dqdt_6= ( -(k*L*cos(q(2))*(q(1)-l_0)) )/I ;



dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4;dqdt_5;dqdt_6];

end


%% 
function [value,isterminal,direction] = end_flight_1(~,q)   % between flight(Apex) and back leg stance           %act on the flight_motion
        % back leg touchdown event
       
		value = q(2) - L*sin(abs(q(3))) - l_0*cos(la(1));
		isterminal = 1;
		direction =  -1;
end


function [value,isterminal,direction] = end_back_stance(~,q)  %between back leg stance and flight stance     %act on the back_stance_motion
	    % back leg liftoff event
       
		value = l_0 - q(1) ;
		isterminal =  1;
		direction =   -1;
end



function [value,isterminal,direction] = end_flight_2(~,q)  %between flight and front leg stance   %act on the flight_motion
        % front leg touchdown event
       
		value =  q(2) - L*sin(abs(q(3))) - l_0*cos(la(2))  ;
		isterminal = 1;
		direction =  -1;
end

function [value,isterminal,direction] = end_front_stance(~,q)  %between front leg stance and flight stance   %act on the front_stance_motion
        % front leg liftoff
       
		value =   l_0 - q(1) ;
		isterminal = 1;
		direction =  -1;
end



function [value,isterminal,direction] = end_flight_3(~,q)          %act on the flight_motion
        % apex event
		value = q(5)  ;
		isterminal = 1;
		direction =  -1;
end


end