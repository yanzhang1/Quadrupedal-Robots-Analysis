function [alpha3,delta,alpha6] = pro_des_theta_thetadot_bs(z,q_0)
%%
g=9.8;                         %gravity acceleration 
k=3520;                         %spring constant
m=20.865;                        %torso mass
l_0=0.323;                       %rest length of leg
L = 0.276;                       % half length of torso
I = 1.3;                         % Torso moment of inertia
%u_0 = [phi_bTD,phi_fTD]
%u_0(1) = [phi_bTD] back leg touchdown angle
%u_0(2) = [phi_fTD] front leg touchdown angle

%q_0 = [y,theta,xdot,thetadot]
%q_0 = [20.0043467600859,0.325610100981765,0.0394615817530123,0.673839000000000,-0.0632172504732866,6.11737300000000];
%q_0 = [0.323000000000000,0.187431221246988,0.039461581753012,-1.872002410776474,-7.132046280147083,6.117373000000000];
%q_0 = [0.323000000000000,0.173431630417578,0.053461172582422,-1.824798448883038,-6.958754726647387,5.826626337509548];
options_back_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_back_stance);

t0 = 0;
time= 10;
%fprintf('%f \n',(1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1));
%% 

 %for i =1:6
   %start at flight phase      
  
  
   %  =[x       ,y       ,theta   ,xdot    ,ydot    ,thetadot
    
  %, l_0 , 0.22689, 0.0129 , 0 , 0.1 , 0.2605];
    [t,q] =ode45(@back_stance_motion, [t0,time] , q_0,options_back_stance);
   %p = q(:,3);
  % delta3 = q(end,3) - q_0(3);
    delta = q(end,2)+ q(end,3) - q_0(2) -  q_0(3);
    s =( q(:,3)+ q(:,2) -q_0(3)-q_0(2))/delta;
     %%%%%%%delta = q(end,2) + q(end,3) - q_0(3)-q_0(2);
   %%%%%%%% s =( q(:,2)+q(:,3) - q_0(2)-q_0(3))/delta;
  % s = 0:0.2:1;
  % z=10;
   n = length(q(:,2));
  %  n = 6;
  %  z = length(q(:,3));
  %  f(1) = q(1,3);
  %  f(2) = q(0.2*z,3);
  %  f(3) = q(0.4*z,3);
   % f(4) = q(0.6*z,3);
   % f(5) = q(0.8*z,3);
   % f(6) = q(1,3);
  %  s =f ;
    A = zeros([n z-1]);
    for i = 1:n 
        for j = 1:z
          A(i,j) = ( factorial(z-1)/(factorial(j-1)*factorial(z - j))) * s(i)^(j-1) *(1-s(i))^(z-j) ; 
      % A(i,j) = s(i)^(j-1);
        end
    end
    
   % alpha_bs = zeros(6,1);
    alpha3 = A\q(:,3);
    alpha6 = A\q(:,6);
 %   alpha5 = A\q(:,5);
   % alpha3 = [alpha_3;q(end,3)];
   % alpha6 = [alpha_6;q(end,6)];
   theta_des3 = zeros(n,1);
   theta_des6 = zeros(n,1);
  for k = 1:z
      for h = 1:n
       theta_des3(h) = theta_des3(h)+ alpha3(k).* ( factorial(z-1)/(factorial(k-1).*factorial(z-k))) .* s(h).^(k-1) .* (1-s(h)).^(z-k);
   %    theta_des6(h) = theta_des6(h)+ alpha6(k).* ( factorial(z-1)/(factorial(k-1).*factorial(z-k))) .* s(h).^(k-1) .* (1-s(h)).^(z-k);
      end    
  end
  
  
 % sas = q(:,3)-theta_des3;
 % error_3 = norm(sas);
 % plot(0:1/(n-1):1,sas,'--');
 % xlabel('n')
 % ylabel('error: $e_{bs}|_{M=10}$', 'Interpreter','latex')
  %  theta_des6 = alpha6(1).*( factorial(5)/(factorial(0).*factorial(5))) .* s.^0 .* (1-s).^5 + alpha6(2).*( factorial(5)/(factorial(1).*factorial(4))) .* s.^1 .* (1-s).^4 + ...
           %      alpha6(3).*( factorial(5)/(factorial(2).*factorial(3))) .* s.^2 .* (1-s).^3 + alpha6(4).*( factorial(5)/(factorial(3).*factorial(2))) .* s.^3 .* (1-s).^2 + ...
           %      alpha6(5).*( factorial(5)/(factorial(4).*factorial(1))) .* s.^4 .* (1-s).^1 + alpha6(6).*( factorial(5)/(factorial(5).*factorial(0))) .* s.^5 .* (1-s).^0 ;         
  %% figure
  % hold on 
  % plot(s,theta_des3,0:1/(z-1):1,alpha3,'o')
  % line(s,theta_des3,'LineStyle','-.','Color','r')
  % xlabel('s')
  % ylabel('b(s)')
  % legend('Polynomial Curve','Coefficients')
 %  plot((0:1/(z-1):1),alpha3,'o');
  % ax1 = gca;
  % ax1.XColor = 'r';
  % ax1.YColor = 'r';
  % ax1_pos = ax1.Position;
 % ax2 = axes('Position',ax1_pos,'XAxisLocation','top','YAxisLocation','right','Color','none');
 
   %line(t,q(:,3),'Parent',ax2,'Color','k')
  % xlabel('time(s)')
  % ylabel('Pitch anlge : $\theta$(rad)', 'Interpreter','latex')
   %legend('Desired Trajectory of Pitch angle')
 %% % title('Degree of polynomial M is', '%d','z-1')       
             % plot(t,q(:,3),'--');
 %  plot(s,theta_des6,'.',(0:0.2:1),alpha6,'o');
   %%%%%%%%%%plot(s,theta_des3,'.',(0:1/(z-1):1),alpha3,'o');
 %    title('Bezier degree five polynomial curve in back stance phase');
  % xlabel('s');
 %  ylabel('b(s)');
    
   %   figure(3)
 %  hold on 
  % plot(t,q(:,2),'--');
 %  plot(t,q(:,7));
   
 %     figure(4)
 %  hold on 
 %  plot(t,q(:,2));
  % plot(t,q(:,8));
    
 %   plot(data(i).q(:,3),data(i).q(:,6));
 %  plot(data(i).q(:,1),data(i).q(:,2));
% xlabel('Time: t(s)'); 
% ylabel('Pitch angle: $\theta$(rad)', 'Interpreter','latex');
 % ylabel('Pitch rate: $\dot{\theta}$(rad/s)', 'Interpreter','latex');
 % title('ICs:[y,$\theta$,$\dot{x}$,$\dot{\theta}$]=[0.3258 m,-0.0000 rad,0.6738 m/s, 6.1173 rad/s]', 'Interpreter','latex');
 % legend('flight phase','back stance','flight phase','flight stance','front phase','flight phase');
 % tx = ('Start point');
 % text(0,3,tx);
  %plot(q_1(1),q_1(2),'*');%,t,q(9),'+');

 %end

%% 




%function dqdt=back_stance_motion(~,q)


%dqdt_1=q(4);
%dqdt_2=q(5);
%dqdt_3=q(6);
%dqdt_4=-(  k*(L^2)*((cos(q(2)))^2) * (q(1)-l_0))/I  - L*sin(q(2))*q(6)^2 + q(1)*(q(5)+q(6))^2 - (k/m)*(q(1)-l_0) - g*cos(q(2)+q(3)) ;
%dqdt_5=-(k*L*cos(q(2))*(q(1)-l_0)*(q(1)-L*sin(q(2))))/(I*q(1))  - (L*cos(q(2))*q(6)^2)/q(1) - 2*(q(4)/q(1))*(q(5)+q(6)) +  (g/q(1))*sin(q(2)+q(3))  ;
%dqdt_6= ( +(k*L*cos(q(2))*(q(1)-l_0)))/I ;


%dqdt=[dqdt_1;dqdt_2;dqdt_3;dqdt_4;dqdt_5;dqdt_6 ];

%end

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





%% 




function [value,isterminal,direction] = end_back_stance(~,q)  %between back leg stance and flight stance     %act on the back_stance_motion
	    % back leg liftoff event
       
		value = l_0 - q(1) ;
		isterminal =  1;
		direction =   -1;
end





end