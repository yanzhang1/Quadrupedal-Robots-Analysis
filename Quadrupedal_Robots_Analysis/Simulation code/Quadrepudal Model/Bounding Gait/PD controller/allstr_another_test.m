function qq=allstr_another_test(Kp,Kd)
%% 
g=9.8;                         %gravity acceleration 
k=3520;                         %spring constant
m=20.865;                        %torso mass
l_0=0.323;                       %rest length of leg
L = 0.276;                       % half length of torso
I = 1.3;
b =55;
%u_0 = [phi_bTD,phi_fTD]
%u_0(1) = [phi_bTD] back leg touchdown angle
%u_0(2) = [phi_fTD] front leg touchdown angle
%0.325814
q_0 =[ 0.33448370392795495620319456975267, 0, 0.69799230392981936876850568296504, 5.2606275978381846769593721546698];
la = [0.226892803  ,0.191986218 ];


%q_0 = [0.3279,  -0.0, 0.6771, 5.9781 ];
%la = [0.2269 ,0.1920];
options_flight_1 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_1);
options_back_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_back_stance);
options_flight_2 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_2);
options_front_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_front_stance);
options_flight_3 = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_flight_3);
tt_end = 0;
time= 100;
t0 = 0;
x_0 = 20;
ydot_0 = 0;
theta_des3 = 0;
theta_des6 = 0;


%fprintf('The total energy at first apex height is : %f \n',(1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1));
%% 
for i = 1:3
  %%
   [t1,q,~,~,events_flight_1] =ode45(@flight_motion, [t0,time] , [x_0, q_0(1), q_0(2) ,q_0(3),  ydot_0 , q_0(4)], options_flight_1);
  %%
   
   time1 = t1 + tt_end ;
   qq(6*i-5).time = time1;
   qq(6*i-5).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
   
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
     time2 = t2 + time1(end);
     qq(6*i-4).time = time2;
     qq(6*i-4).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
  
    if events_back_stance == 1 %(back leg liftoff)    % proceed back leg stance phase to flight phase
    
        q_2=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
        %q_2 = [0.323000000000000	-0.401851826074128	0.1927000000	1.66315225464006	3.31056565845562	-3.98101826814071];
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
       qq(6*i-3).time = time3;
       qq(6*i-3).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
        if events_flight_2 == 1 %( reach apex height)   % proceed flight to apex
         q_3=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
         
         %  =[x       ,y,      ,theta   ,xdot,   ,ydot    ,thetadot]
         time =time - t3(end);
         
        [t4,q,~,~,events_flight_3] =ode45(@flight_motion, [t0,time] ,[q_3(1),q_3(2),q_3(3),q_3(4),q_3(5),q_3(6)] , options_flight_2);
        time4 = t4 +time3(end);
        qq(6*i-2).time = time4; 
        qq(6*i-2).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
        
        
        
        
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
              qq(6*i-1).time = time5 ;
              qq(6*i-1).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
  
               if events_front_stance == 1 %(front leg liftoff)   % proceed front leg stance phase to flight phase
              q_5=[q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];       
               
              time =time - t5(end);
              %%
              
               x4_0 = x_ftoe - L*cos(q_5(3)) - q_5(1)*sin(q_5(2)+q_5(3))  ;
               y4_0 = -  L*sin(q_5(3)) + q_5(1)*cos(q_5(2)+q_5(3)) ;
               theta4_0 = q_5(3); 
               xdot4_0 = - q_5(4)*sin(q_5(2)+q_5(3)) - q_5(1)*cos(q_5(2)+q_5(3))*(q_5(5)+q_5(6)) + L*q_5(6)*sin(q_5(3));
               ydot4_0 =   q_5(4)*cos(q_5(2)+q_5(3)) - q_5(1)*sin(q_5(2)+q_5(3))*(q_5(5)+q_5(6)) - L*q_5(6)*cos(q_5(3));
               thetadot4_0 = q_5(6) ;
              
              %%
            
                  [t6,q,~,~,~] =ode45(@flight_motion, [t0,time] , [x4_0 , y4_0 , theta4_0 , xdot4_0 , ydot4_0 ,thetadot4_0] , options_flight_3);
                  time6 = t6 + time5(end) ;
                  qq(6*i).time = time6 ;
                  qq(6*i).state = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];
                   q_6 = [q(end,1),q(end,2),q(end,3),q(end,4),q(end,5),q(end,6)];
                   q_7=[q_6(2),q_6(3),q_6(4),q_6(6)];
       
               %   fprintf('The total energy at next apex height is : %f \n',(1/2)*m*(q(4)^2 + q(5)^2) + (1/2)*I*q(6)^2 + m*g*q(2));
             
               time =time - t6(end);
               tt_end = time6(end);
               x_0 = q_6(1);
               ydot_0 = q_6(5);
               q_0 = q_7;
               t0 = t0 + 0.00000001;
               %   fprintf('%f \n',((1/2)*m*(q(4)^2 + q(5)^2) + (1/2)*I*q(6)^2 + m*g*q(2))-((1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1)));
               end
       
        end
  
        end
  
    end
   end
end
%   figure
 %  hold on
% for i =1:6
%   plot(qq(i).time,qq(i).state(:,3));
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
[alpha3,delta,alpha6] = pro_des_theta_thetadot_bs(17,backstance_0);
%delta1 = delta(1);
%delta2 = delta(2);
%delta3 = delta(3);
%delta4 = delta(4);
%delta5 = delta(5);
%delta6 = delta(6);
%alpha1 = alpha(:,1);
%alpha2 = alpha(:,2);
%alpha3 = alpha(:,3);
%alpha3 = [0.0467; 0.2000;0.3500;0.4500;0.3500;0.1984];
%alpha4 = alpha(:,4);
%alpha5 = alpha(:,5);
%alpha6 = alpha(:,6);
%s1 = (q(1) - backstance_0(1)) /delta1;
%s2 = (q(2) - backstance_0(2)) /delta2;
s3 = (q(3)+q(2)-backstance_0(2) - backstance_0(3)) /delta;
%s4 = (q(4) - backstance_0(4)) /delta4;
%s5 = (q(5) - backstance_0(5)) /delta5;
%s6 = (q(6) - backstance_0(6)) /delta6;
%theta_des1 = alpha1(1)*( factorial(5)/(factorial(0)*factorial(5))) * s1^0 * (1-s1)^5 + alpha1(2)*( factorial(5)/(factorial(1)*factorial(4))) * s1^1 * (1-s1)^4 + ...
 %            alpha1(3)*( factorial(5)/(factorial(2)*factorial(3))) * s1^2 * (1-s1)^3 + alpha1(4)*( factorial(5)/(factorial(3)*factorial(2))) * s1^3 * (1-s1)^2 + ...
  %           alpha1(5)*( factorial(5)/(factorial(4)*factorial(1))) * s1^4 * (1-s1)^1 + alpha1(6)*( factorial(5)/(factorial(5)*factorial(0))) * s1^5 * (1-s1)^0 ; 
%theta_des2 = alpha2(1)*( factorial(5)/(factorial(0)*factorial(5))) * s2^0 * (1-s2)^5 + alpha2(2)*( factorial(5)/(factorial(1)*factorial(4))) * s2^1 * (1-s2)^4 + ...
 %            alpha2(3)*( factorial(5)/(factorial(2)*factorial(3))) * s2^2 * (1-s2)^3 + alpha2(4)*( factorial(5)/(factorial(3)*factorial(2))) * s2^3 * (1-s2)^2 + ...
  %           alpha2(5)*( factorial(5)/(factorial(4)*factorial(1))) * s2^4 * (1-s2)^1 + alpha2(6)*( factorial(5)/(factorial(5)*factorial(0))) * s2^5 * (1-s2)^0 ; 
theta_des3 = alpha3(1)*( factorial(16)/(factorial(0)*factorial(16))) * s3^0 * (1-s3)^16 + alpha3(2)*( factorial(16)/(factorial(1)*factorial(15))) * s3^1 * (1-s3)^15 + ...
             alpha3(3)*( factorial(16)/(factorial(2)*factorial(14))) * s3^2 * (1-s3)^14 + alpha3(4)*( factorial(16)/(factorial(3)*factorial(13))) * s3^3 * (1-s3)^13 + ...
             alpha3(5)*( factorial(16)/(factorial(4)*factorial(12))) * s3^4 * (1-s3)^12 + alpha3(6)*( factorial(16)/(factorial(5)*factorial(11))) * s3^5 * (1-s3)^11 + ...
             alpha3(7)*( factorial(16)/(factorial(6)*factorial(10))) * s3^6 * (1-s3)^10 + alpha3(8)*( factorial(16)/(factorial(7)*factorial(9))) * s3^7 * (1-s3)^9 + ...
             alpha3(9)*( factorial(16)/(factorial(8)*factorial(8))) * s3^8 * (1-s3)^8 + alpha3(10)*( factorial(16)/(factorial(9)*factorial(7))) * s3^9 * (1-s3)^7+ ...
             alpha3(11)*( factorial(16)/(factorial(10)*factorial(6))) * s3^10 * (1-s3)^6 + alpha3(12)*( factorial(16)/(factorial(11)*factorial(5))) * s3^11 * (1-s3)^5 + ...
             alpha3(13)*( factorial(16)/(factorial(12)*factorial(4))) * s3^12 * (1-s3)^4 + alpha3(14)*( factorial(16)/(factorial(13)*factorial(3))) * s3^13 * (1-s3)^3 + ...
             alpha3(15)*( factorial(16)/(factorial(14)*factorial(2))) * s3^14 * (1-s3)^2 + alpha3(16)*( factorial(16)/(factorial(15)*factorial(1))) * s3^15 * (1-s3)^1 + ...
             alpha3(17)*( factorial(16)/(factorial(16)*factorial(0))) * s3^16 * (1-s3)^0  ;%+ ...
%theta_des4 = alpha4(1)*( factorial(5)/(factorial(0)*factorial(5))) * s4^0 * (1-s4)^5 + alpha4(2)*( factorial(5)/(factorial(1)*factorial(4))) * s4^1 * (1-s4)^4 + ...
 %            alpha4(3)*( factorial(5)/(factorial(2)*factorial(3))) * s4^2 * (1-s4)^3 + alpha4(4)*( factorial(5)/(factorial(3)*factorial(2))) * s4^3 * (1-s4)^2 + ...
  %           alpha4(5)*( factorial(5)/(factorial(4)*factorial(1))) * s4^4 * (1-s4)^1 + alpha4(6)*( factorial(5)/(factorial(5)*factorial(0))) * s4^5 * (1-s4)^0 ; 
%theta_des5 = alpha5(1)*( factorial(5)/(factorial(0)*factorial(5))) * s3^0 * (1-s3)^5 + alpha5(2)*( factorial(5)/(factorial(1)*factorial(4))) * s3^1 * (1-s3)^4 + ...
    %         alpha5(3)*( factorial(5)/(factorial(2)*factorial(3))) * s3^2 * (1-s3)^3 + alpha5(4)*( factorial(5)/(factorial(3)*factorial(2))) * s3^3 * (1-s3)^2 + ...
    %         alpha5(5)*( factorial(5)/(factorial(4)*factorial(1))) * s3^4 * (1-s3)^1 + alpha5(6)*( factorial(5)/(factorial(5)*factorial(0))) * s3^5 * (1-s3)^0 ; 
theta_des6 =  alpha6(1)*( factorial(16)/(factorial(0)*factorial(16))) * s3^0 * (1-s3)^16 + alpha6(2)*( factorial(16)/(factorial(1)*factorial(15))) * s3^1 * (1-s3)^15 + ...
             alpha6(3)*( factorial(16)/(factorial(2)*factorial(14))) * s3^2 * (1-s3)^14 + alpha6(4)*( factorial(16)/(factorial(3)*factorial(13))) * s3^3 * (1-s3)^13 + ...
             alpha6(5)*( factorial(16)/(factorial(4)*factorial(12))) * s3^4 * (1-s3)^12 + alpha6(6)*( factorial(16)/(factorial(5)*factorial(11))) * s3^5 * (1-s3)^11 + ...
             alpha6(7)*( factorial(16)/(factorial(6)*factorial(10))) * s3^6 * (1-s3)^10 + alpha6(8)*( factorial(16)/(factorial(7)*factorial(9))) * s3^7 * (1-s3)^9 + ...
             alpha6(9)*( factorial(16)/(factorial(8)*factorial(8))) * s3^8 * (1-s3)^8 + alpha6(10)*( factorial(16)/(factorial(9)*factorial(7))) * s3^9 * (1-s3)^7+ ...
             alpha6(11)*( factorial(16)/(factorial(10)*factorial(6))) * s3^10 * (1-s3)^6 + alpha6(12)*( factorial(16)/(factorial(11)*factorial(5))) * s3^11 * (1-s3)^5 + ...
             alpha6(13)*( factorial(16)/(factorial(12)*factorial(4))) * s3^12 * (1-s3)^4 + alpha6(14)*( factorial(16)/(factorial(13)*factorial(3))) * s3^13 * (1-s3)^3 + ...
             alpha6(15)*( factorial(16)/(factorial(14)*factorial(2))) * s3^14 * (1-s3)^2 + alpha6(16)*( factorial(16)/(factorial(15)*factorial(1))) * s3^15 * (1-s3)^1 + ...
             alpha6(17)*( factorial(16)/(factorial(16)*factorial(0))) * s3^16 * (1-s3)^0  ;
%  theta_des3 = zeros(n,1);
%  theta_des6 = zeros(n,1);

 % for b = 1:z
     
  %     theta_des3 = theta_des3 + alpha3(b).* ( factorial(z-1)/(factorial(b-1).*factorial(z-b))) .* s3.^(b-1) .* (1-s3).^(z-b);
  %     theta_des6 = theta_des6 + alpha6(b).* ( factorial(z-1)/(factorial(b-1).*factorial(z-b))) .* s3.^(b-1) .* (1-s3).^(z-b);
          
 % end                        
%u1 = -Kp* (q(1) - theta_des1) - Kd * (q(4));       
%u2 = -Kp* (q(2)-  theta_des2) - Kd *( q(5) );       
%u3 = -Kp* (q(3)-  theta_des3) - Kd *( q(6));
%theta_des1 = 0.323;
%theta_des2 = -0.401851826;
%theta_des3 = 0.2098655672;
%pbps = (alpha3(2)-alpha3(1))*( factorial(5)/(factorial(0)*factorial(4))) * s3^0 * (1-s3)^4 +(alpha3(3)-alpha3(2))*( factorial(5)/(factorial(1)*factorial(3))) * s3^1 * (1-s3)^3 + ...
 %      (alpha3(4)-alpha3(3))*( factorial(5)/(factorial(2)*factorial(2))) * s3^2 * (1-s3)^2 + (alpha3(5)-alpha3(4))*( factorial(5)/(factorial(3)*factorial(1))) * s3^3 * (1-s3)^1 + ...
 %      (alpha3(6)-alpha3(5))*( factorial(5)/(factorial(4)*factorial(0))) * s3^4 * (1-s3)^0 ;
%nnn = (pbps/delta)*(theta_des5 + theta_des6);
%%%%%   - ( k*L*cos(q(2))*(q(1)-l_0) )/I

%u1 = 0;
%u1 =  ((I*q(1))/(L*cos(q(2))*(q(1) - L*sin(q(2))))) *( (k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0))/I                               + L*sin(q(2))*q(6)^2 - q(1)*(q(5)+q(6))^2 +(k/m)*(q(1)-l_0) + g*cos(q(2)+q(3)) -Kp* (q(1)-theta_des1) - Kd *  q(4)     )     ;
%u2 =      ((k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))                           +(L*cos(q(2))*q(6)^2)/q(1) + 2*(q(4)/q(1))*(q(5)+q(6)) -  (g/q(1))*sin(q(2)+q(3)) -Kp* (q(2)-theta_des2) - Kd * q(5))*((m*I*q(1)^2)/(m*(q(1)-L*sin(q(2)))^2+I));
%u3 = ((I*q(1))/(q(1)-L*sin(q(2))))*(Kp* (q(3)-theta_des3) + Kd * (q(6)) + ( k*L*cos(q(2))*(q(1)-l_0) )/I);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%u3 = ((I*q(1))/(q(1)-L*sin(q(2))))*(  Kp* (q(3)-  theta_des3) + Kd *( q(6) - theta_des6) + ( k*L*cos(q(2))*(q(1)-l_0) )/I);
u3 =  Kp* (q(3)-  theta_des3) + Kd *( q(6) - theta_des6);
dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4=-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u3)/q(1)    )/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
dqdt_5=-(k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))  + ( ((q(1)-L*sin(q(2)))^2)/(I*q(1)^2)  + 1/(m*(q(1)^2)))*u3 - (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
dqdt_6= ( +(k*L*cos(q(2))*(q(1)-l_0)) - (( u3*(q(1)-L*sin(q(2))))/q(1)) )/I ;
%dqdt_4 = u1;
%dqdt_5 = u2;
%dqdt_6 = u3;
%dqdt_4= %-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u1)/q(1)    )/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
%dqdt_5=-Kp* (q(2)-theta_des2) - Kd * q(5);
%dqdt_6=-Kp* (q(3)-theta_des3) - Kd * q(6);
%( k*L*cos(q(2))*(q(1)-l_0) - (( u3*(q(1)-L*sin(q(2))))/q(1)) )/I
%%%%%-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u2)/q(1)    )/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
%%%%%-(k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))  + (((q(1)-L*sin(q(2)))^2)/(I*q(1)^2)  + 1/(m*(q(1)^2)))*u2 - (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3));
%%%%%( k*L*cos(q(2))*(q(1)-l_0) - (( u2*(q(1)-L*sin(q(2))))/q(1)) )/I ;
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
[alpha3,delta,alpha6] = pro_des_theta_thetadot_fs(17,frontstance_0);
%delta1 = delta(1);
%delta2 = delta(2);
%delta3 = delta(3);
%alpha3 = [-0.2099;-0.35;-0.4;-0.35;-0.2;-0.0395];
%delta4 = delta(4);
%delta5 = delta(5);
%delta6 = delta(6);
%alpha1 = alpha(:,1);
%alpha2 = alpha(:,2);
%alpha3 = alpha(:,3);
%alpha4 = alpha(:,4);
%alpha5 = alpha(:,5);
%alpha6 = alpha(:,6);
%s1 = (q(1) - frontstance_0(1)) /delta1;
%s2 = (q(2) -frontstance_0(2)) /delta2;
s3 = (q(3)+q(2)-frontstance_0(2) - frontstance_0(3)) /delta;
%s4 = (q(4) - frontstance_0(4)) /delta4;
%s5 = (q(5) - frontstance_0(5)) /delta5;
%s6 = (q(6) - frontstance_0(6)) /delta6;
%theta_des1 = alpha1(1)*( factorial(5)/(factorial(0)*factorial(5))) * s1^0 * (1-s1)^5 + alpha1(2)*( factorial(5)/(factorial(1)*factorial(4))) * s1^1 * (1-s1)^4 + ...
 %            alpha1(3)*( factorial(5)/(factorial(2)*factorial(3))) * s1^2 * (1-s1)^3 + alpha1(4)*( factorial(5)/(factorial(3)*factorial(2))) * s1^3 * (1-s1)^2 + ...
  %           alpha1(5)*( factorial(5)/(factorial(4)*factorial(1))) * s1^4 * (1-s1)^1 + alpha1(6)*( factorial(5)/(factorial(5)*factorial(0))) * s1^5 * (1-s1)^0 ; 
%theta_des2 = alpha2(1)*( factorial(5)/(factorial(0)*factorial(5))) * s2^0 * (1-s2)^5 + alpha2(2)*( factorial(5)/(factorial(1)*factorial(4))) * s2^1 * (1-s2)^4 + ...
 %            alpha2(3)*( factorial(5)/(factorial(2)*factorial(3))) * s2^2 * (1-s2)^3 + alpha2(4)*( factorial(5)/(factorial(3)*factorial(2))) * s2^3 * (1-s2)^2 + ...
  %           alpha2(5)*( factorial(5)/(factorial(4)*factorial(1))) * s2^4 * (1-s2)^1 + alpha2(6)*( factorial(5)/(factorial(5)*factorial(0))) * s2^5 * (1-s2)^0 ; 
%theta_des3 = alpha3(1)*( factorial(5)/(factorial(0)*factorial(5))) * s3^0 * (1-s3)^5 + alpha3(2)*( factorial(5)/(factorial(1)*factorial(4))) * s3^1 * (1-s3)^4 + ...
 %            alpha3(3)*( factorial(5)/(factorial(2)*factorial(3))) * s3^2 * (1-s3)^3 + alpha3(4)*( factorial(5)/(factorial(3)*factorial(2))) * s3^3 * (1-s3)^2 + ...
 %            alpha3(5)*( factorial(5)/(factorial(4)*factorial(1))) * s3^4 * (1-s3)^1 + alpha3(6)*( factorial(5)/(factorial(5)*factorial(0))) * s3^5 * (1-s3)^0 ;
%theta_des4 = alpha4(1)*( factorial(5)/(factorial(0)*factorial(5))) * s4^0 * (1-s4)^5 + alpha4(2)*( factorial(5)/(factorial(1)*factorial(4))) * s4^1 * (1-s4)^4 + ...
 %            alpha4(3)*( factorial(5)/(factorial(2)*factorial(3))) * s4^2 * (1-s4)^3 + alpha4(4)*( factorial(5)/(factorial(3)*factorial(2))) * s4^3 * (1-s4)^2 + ...
  %           alpha4(5)*( factorial(5)/(factorial(4)*factorial(1))) * s4^4 * (1-s4)^1 + alpha4(6)*( factorial(5)/(factorial(5)*factorial(0))) * s4^5 * (1-s4)^0 ; 
%theta_des5 = alpha5(1)*( factorial(5)/(factorial(0)*factorial(5))) * s3^0 * (1-s3)^5 + alpha5(2)*( factorial(5)/(factorial(1)*factorial(4))) * s3^1 * (1-s3)^4 + ...
 %            alpha5(3)*( factorial(5)/(factorial(2)*factorial(3))) * s3^2 * (1-s3)^3 + alpha5(4)*( factorial(5)/(factorial(3)*factorial(2))) * s3^3 * (1-s3)^2 + ...
 %            alpha5(5)*( factorial(5)/(factorial(4)*factorial(1))) * s3^4 * (1-s3)^1 + alpha5(6)*( factorial(5)/(factorial(5)*factorial(0))) * s3^5 * (1-s3)^0 ; 
%%theta_des6 = alpha6(1)*( factorial(5)/(factorial(0)*factorial(5))) * s3^0 * (1-s3)^5 + alpha6(2)*( factorial(5)/(factorial(1)*factorial(4))) * s3^1 * (1-s3)^4 + ...
 %            alpha6(3)*( factorial(5)/(factorial(2)*factorial(3))) * s3^2 * (1-s3)^3 + alpha6(4)*( factorial(5)/(factorial(3)*factorial(2))) * s3^3 * (1-s3)^2 + ...
 %            alpha6(5)*( factorial(5)/(factorial(4)*factorial(1))) * s3^4 * (1-s3)^1 + alpha6(6)*( factorial(5)/(factorial(5)*factorial(0))) * s3^5 * (1-s3)^0 ;
%pbps = (alpha3(2)-alpha3(1))*( factorial(5)/(factorial(0)*factorial(4))) * s3^0 * (1-s3)^4 +(alpha3(3)-alpha3(2))*( factorial(5)/(factorial(1)*factorial(3))) * s3^1 * (1-s3)^3 + ...
 %      (alpha3(4)-alpha3(3))*( factorial(5)/(factorial(2)*factorial(2))) * s3^2 * (1-s3)^2 + (alpha3(5)-alpha3(4))*( factorial(5)/(factorial(3)*factorial(1))) * s3^3 * (1-s3)^1 + ...
 %      (alpha3(6)-alpha3(5))*( factorial(5)/(factorial(4)*factorial(0))) * s3^4 * (1-s3)^0 ;
%nnn = (pbps/delta)*(theta_des5 + theta_des6);              
%u1 = -Kp* (q(1) - theta_des1) - Kd * (q(4));       
%u2 = -Kp* (q(2)-  theta_des2) - Kd *( q(5) );       
%u3 = -Kp* (q(3)-  theta_des3) - Kd *( q(6));
%theta_des1 = 0.323;
%theta_des2 = -0.401851826;
%theta_des3 = 0.2098655672;

theta_des3 = alpha3(1)*( factorial(16)/(factorial(0)*factorial(16))) * s3^0 * (1-s3)^16 + alpha3(2)*( factorial(16)/(factorial(1)*factorial(15))) * s3^1 * (1-s3)^15 + ...
             alpha3(3)*( factorial(16)/(factorial(2)*factorial(14))) * s3^2 * (1-s3)^14 + alpha3(4)*( factorial(16)/(factorial(3)*factorial(13))) * s3^3 * (1-s3)^13 + ...
             alpha3(5)*( factorial(16)/(factorial(4)*factorial(12))) * s3^4 * (1-s3)^12 + alpha3(6)*( factorial(16)/(factorial(5)*factorial(11))) * s3^5 * (1-s3)^11 + ...
             alpha3(7)*( factorial(16)/(factorial(6)*factorial(10))) * s3^6 * (1-s3)^10 + alpha3(8)*( factorial(16)/(factorial(7)*factorial(9))) * s3^7 * (1-s3)^9 + ...
             alpha3(9)*( factorial(16)/(factorial(8)*factorial(8))) * s3^8 * (1-s3)^8 + alpha3(10)*( factorial(16)/(factorial(9)*factorial(7))) * s3^9 * (1-s3)^7+ ...
             alpha3(11)*( factorial(16)/(factorial(10)*factorial(6))) * s3^10 * (1-s3)^6 + alpha3(12)*( factorial(16)/(factorial(11)*factorial(5))) * s3^11 * (1-s3)^5 + ...
             alpha3(13)*( factorial(16)/(factorial(12)*factorial(4))) * s3^12 * (1-s3)^4 + alpha3(14)*( factorial(16)/(factorial(13)*factorial(3))) * s3^13 * (1-s3)^3 + ...
             alpha3(15)*( factorial(16)/(factorial(14)*factorial(2))) * s3^14 * (1-s3)^2 + alpha3(16)*( factorial(16)/(factorial(15)*factorial(1))) * s3^15 * (1-s3)^1 + ...
             alpha3(17)*( factorial(16)/(factorial(16)*factorial(0))) * s3^16 * (1-s3)^0  ;%+ ...
%theta_des4 = alpha4(1)*( factorial(5)/(factorial(0)*factorial(5))) * s4^0 * (1-s4)^5 + alpha4(2)*( factorial(5)/(factorial(1)*factorial(4))) * s4^1 * (1-s4)^4 + ...
 %            alpha4(3)*( factorial(5)/(factorial(2)*factorial(3))) * s4^2 * (1-s4)^3 + alpha4(4)*( factorial(5)/(factorial(3)*factorial(2))) * s4^3 * (1-s4)^2 + ...
  %           alpha4(5)*( factorial(5)/(factorial(4)*factorial(1))) * s4^4 * (1-s4)^1 + alpha4(6)*( factorial(5)/(factorial(5)*factorial(0))) * s4^5 * (1-s4)^0 ; 
%theta_des5 = alpha5(1)*( factorial(5)/(factorial(0)*factorial(5))) * s3^0 * (1-s3)^5 + alpha5(2)*( factorial(5)/(factorial(1)*factorial(4))) * s3^1 * (1-s3)^4 + ...
    %         alpha5(3)*( factorial(5)/(factorial(2)*factorial(3))) * s3^2 * (1-s3)^3 + alpha5(4)*( factorial(5)/(factorial(3)*factorial(2))) * s3^3 * (1-s3)^2 + ...
    %         alpha5(5)*( factorial(5)/(factorial(4)*factorial(1))) * s3^4 * (1-s3)^1 + alpha5(6)*( factorial(5)/(factorial(5)*factorial(0))) * s3^5 * (1-s3)^0 ; 
theta_des6 =  alpha6(1)*( factorial(16)/(factorial(0)*factorial(16))) * s3^0 * (1-s3)^16 + alpha6(2)*( factorial(16)/(factorial(1)*factorial(15))) * s3^1 * (1-s3)^15 + ...
             alpha6(3)*( factorial(16)/(factorial(2)*factorial(14))) * s3^2 * (1-s3)^14 + alpha6(4)*( factorial(16)/(factorial(3)*factorial(13))) * s3^3 * (1-s3)^13 + ...
             alpha6(5)*( factorial(16)/(factorial(4)*factorial(12))) * s3^4 * (1-s3)^12 + alpha6(6)*( factorial(16)/(factorial(5)*factorial(11))) * s3^5 * (1-s3)^11 + ...
             alpha6(7)*( factorial(16)/(factorial(6)*factorial(10))) * s3^6 * (1-s3)^10 + alpha6(8)*( factorial(16)/(factorial(7)*factorial(9))) * s3^7 * (1-s3)^9 + ...
             alpha6(9)*( factorial(16)/(factorial(8)*factorial(8))) * s3^8 * (1-s3)^8 + alpha6(10)*( factorial(16)/(factorial(9)*factorial(7))) * s3^9 * (1-s3)^7+ ...
             alpha6(11)*( factorial(16)/(factorial(10)*factorial(6))) * s3^10 * (1-s3)^6 + alpha6(12)*( factorial(16)/(factorial(11)*factorial(5))) * s3^11 * (1-s3)^5 + ...
             alpha6(13)*( factorial(16)/(factorial(12)*factorial(4))) * s3^12 * (1-s3)^4 + alpha6(14)*( factorial(16)/(factorial(13)*factorial(3))) * s3^13 * (1-s3)^3 + ...
             alpha6(15)*( factorial(16)/(factorial(14)*factorial(2))) * s3^14 * (1-s3)^2 + alpha6(16)*( factorial(16)/(factorial(15)*factorial(1))) * s3^15 * (1-s3)^1 + ...
             alpha6(17)*( factorial(16)/(factorial(16)*factorial(0))) * s3^16 * (1-s3)^0  ;
     
  %     theta_des3 = theta_des3+ alpha3(b).* ( factorial(z-1)/(factorial(b-1).*factorial(z-b))) .* s3.^(b-1) .* (1-s3).^(z-b);
  %     theta_des6 = theta_des6+ alpha6(b).* ( factorial(z-1)/(factorial(b-1).*factorial(z-b))) .* s3.^(b-1) .* (1-s3).^(z-b);
          
 % end  

%%%%%   - ( k*L*cos(q(2))*(q(1)-l_0) )/I
%u1 = 0 ;
%u1 = -((I*q(1))/(L*cos(q(2))*(q(1)+L*sin(q(2)))))*(- L*sin(q(2))*q(6)^2 - q(1)*(q(5)+q(6))^2 + (k/m)*(q(1)-l_0) + g*cos(q(2)+q(3)) + (k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0))/I -Kp * (q(1)-theta_des1) - Kd * q(4)) ;
%u2 = ((m*I*q(1)^2)/(m*(q(1)+L*sin(q(2)))^2+I)  )*(-(L*cos(q(2))*q(6)^2)/q(1) + 2*(q(4)/q(1))*(q(5)+q(6)) -  (g/q(1))*sin(q(2)+q(3)) -(k*L*cos(q(2))*(q(1)-l_0)*(q(1)+L*sin(q(2))))/(I*q(1)) -Kp * (q(2)-theta_des2) - Kd * q(5) ) ;
%%%%%%%%%%%%%%%u3 = ((I*q(1))/(q(1)+L*sin(q(2))))*(+(Kp * (q(3)-theta_des3) + Kd * (q(6) - theta_des6) )  -( (k*L*cos(q(2))*(q(1)-l_0))/I ) );
%theta_des = alpha(1) + alpha(2)*s + alpha(3)*s^2 +alpha(4)*s^3 + alpha(5)*s^4 + alpha(6)*s^5;
%u1 = ((I*q(1))/(q(1)+L*sin(q(2))))*((Kp * (q(3)-theta_des) + Kd * q(6))   -( (k*L*cos(q(2))*(q(1)-l_0))/I ) );
%u2_star = ((I*q(1))/(q(1)+L*sin(q(2))))*(-(k*(q(1)-l_0)*L*cos(q(2)))/I + (L*cos(q(2))*u1)/I);
u3 = Kp * (q(3)-theta_des3) + Kd * (q(6) - theta_des6);

dqdt_1=q(4);
dqdt_2=q(5);
dqdt_3=q(6);
dqdt_4=( - k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0) - (L*cos(q(2))*(q(1)+L*sin(q(2)))*u3)/q(1)    )/I  + L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
dqdt_5=(k*L*cos(q(2))*(q(1)-l_0)*(q(1)+L*sin(q(2))))/(I*q(1))  + ( ((q(1)+L*sin(q(2)))^2)/(I*(q(1)^2))  + 1/(m*(q(1)^2)))*u3  + (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
dqdt_6= ( -(k*L*cos(q(2))*(q(1)-l_0)) - (( u3*(q(1)+L*sin(q(2))))/q(1)) )/I ;

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