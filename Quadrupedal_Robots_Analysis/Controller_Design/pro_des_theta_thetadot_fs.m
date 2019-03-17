function [alpha3,delta,alpha6] = pro_des_theta_thetadot_fs(z,q_0)
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
%q_0 = [0.323000000000000,0.401851590928573,-0.209865372928573,-1.66315325899413,3.31056539930155,-3.98101826814071];
%q_0 = [0.323000000000000,0.377018039088918,-0.185031821088918,-1.635738531966014,3.369243969854467,-4.126988353193027];
options_front_stance = odeset('RelTol',1e-10,'AbsTol',1e-9,'Events', @end_front_stance);

t0 = 0;
time= 10;
%fprintf('%f \n',(1/2)*m*(q_0(3)^2) + (1/2)*I*q_0(4)^2 + m*g*q_0(1));
%% 

 %for i =1:6

    [~,q] =ode45(@front_stance_motion, [t0,time] , q_0,options_front_stance);
   

  
   % delta = q(end,3) -q_0(3);
  delta = q(end,2) + q(end,3) - q_0(2) - q_0(3);
    s3 =( q(:,3)+q(:,2) -q_0(3) - q_0(2))/delta;
 % s3 = (q(:,3) - q_0(3)) /delta;
 %  z = 13;
   n = length(q(:,2));
 
    A3 = zeros(n,z-1);
    for i = 1:n 
        for j = 1:z
          A3(i,j) = ( factorial(z-1)/(factorial(j-1)*factorial(z - j))) * s3(i)^(j-1) *(1-s3(i))^(z-j) ; 
        %A3(i,j) = s3(i).^(j-1);
        end
        
    end
    
    
   % alpha_bs = zeros(6,1);
   % alpha1 = A3\q(:,1);
   % alpha2 = A3\q(:,2);
   % alpha4 = A3\q(:,4);
   % alpha5 = A3\q(:,5);
    alpha3 = pinv(A3)*(q(:,3));
   % alpha3 = [alpha_3;q(end,3)];
    alpha6 = A3\q(:,6);
  %  alpha6 = [alpha_6;q(end,6)];
  theta_des3 = zeros(n,1);
  % mmmm = -0.0394610504798571 - alpha3(end);
 % theta_des3 = alpha3(1).*( factorial(16)/(factorial(0).*factorial(16))) .* s3.^0 .* (1-s3).^16 + alpha3(2).*( factorial(16)/(factorial(1).*factorial(15))) .* s3.^1 .* (1-s3).^15 + ...
      %       alpha3(3).*( factorial(16)/(factorial(2).*factorial(14))) .* s3.^2 .* (1-s3).^14 + alpha3(4).*( factorial(16)/(factorial(3).*factorial(13))) .* s3.^3 .* (1-s3).^13 + ...
      %       alpha3(5).*( factorial(16)/(factorial(4).*factorial(12))) .* s3.^4 .* (1-s3).^12 + alpha3(6).*( factorial(16)/(factorial(5).*factorial(11))) .* s3.^5 .* (1-s3).^11 + ...
      %       alpha3(7).*( factorial(16)/(factorial(6).*factorial(10))) .* s3.^6 .* (1-s3).^10 + alpha3(8).*( factorial(16)/(factorial(7).*factorial(9))) .* s3.^7 .* (1-s3).^9 + ...
      %       alpha3(9).*( factorial(16)/(factorial(8).*factorial(8))) .* s3.^8 .* (1-s3).^8 + alpha3(10).*( factorial(16)/(factorial(9).*factorial(7))) .* s3.^9 .* (1-s3).^7+ ...
      %       alpha3(11).*( factorial(16)/(factorial(10).*factorial(6))) .* s3.^10 .* (1-s3).^6 + alpha3(12).*( factorial(16)/(factorial(11).*factorial(5))) .* s3.^11 .* (1-s3).^5 + ...
      %       alpha3(13).*( factorial(16)/(factorial(12).*factorial(4))) .* s3.^12 .* (1-s3).^4 + alpha3(14).*( factorial(16)/(factorial(13).*factorial(3))) .* s3.^13 .* (1-s3).^3 + ...
      %       alpha3(15).*( factorial(16)/(factorial(14).*factorial(2))) .* s3.^14 .* (1-s3).^2 + alpha3(16).*( factorial(16)/(factorial(15).*factorial(1))) .* s3.^15 .* (1-s3).^1 + ...
      %       alpha3(17).*( factorial(16)/(factorial(16).*factorial(0))) .* s3.^16 .* (1-s3).^0  ;
  for h = 1:n
      for k = 1:z
     theta_des3(h) = theta_des3(h)+ alpha3(k)* ( factorial(z-1)/(factorial(k-1)*factorial(z-k))) * s3(h)^(k-1) * (1-s3(h))^(z-k);
   %   theta_des3(h) = theta_des3(h) + alpha3(k) .* s3(h).^(k-1);
      end    
  end
  %yyy = theta_des3;
  %xxx = q(:,3);
  
 % sas = theta_des3 - q(:,3);
  %plot(sas);
 % error_3 = norm(sas);
 
    %   figure(1)
   %    hold on
 %  figure
  % hold on 
 %  plot(s3,theta_des3,0:1/(z-1):1,alpha3,'o')
  % line(t,theta_des3,'LineStyle','-.','Color','r')
 %  xlabel('s')
 %  ylabel('b(s)')
 % legend('Polynomial Curve','Coefficients')
%  plot((0:1/(z-1):1),alpha3,'o');
 %  ax1 = gca;
 %  ax1.XColor = 'r';
  % ax1.YColor = 'r';
  % ax1_pos = ax1.Position;
  % ax2 = axes('Position',ax1_pos,'XAxisLocation','top','YAxisLocation','right','Color','none');
 
 % line(t,q(:,3),'Parent',ax2,'Color','k')
%  xlabel('time(s)')
 % ylabel('Pitch anlge : $\theta$(rad)', 'Interpreter','latex')
 % legend('Desired Trajectory of Pitch angle')
 %%  title('Degree of polynomial M is 5') 
      %  plot(s3,theta_des3,'.',(0:0.2:1),alpha3,'o');
        %  plot(s3,theta_des6,'.',(0:0.2:1),alpha6,'o');
  %  title('Bezier degree five polynomial curve in front stance phase');
%    xlabel('s');
  %  ylabel('b(s)');
    
  %  alpha = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6];
    
    
    
    
    
    
  % s1 = (q(:,2) - q_0(2))/(q(end,2)- q_0(2));
   % alpha_fs = [alpha_bs1 ,alpha_bs2,alpha_bs3 ,alpha_bs4 ,alpha_bs5,alpha_bs6 ];
   % q_1 = [q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6)];%,q(:,7),q(:,8),q(:,9),q(:,10),q(:,11),q(:,12)];
   % time = time - t(end);
 %  figure(1)
%   hold on
 %   plot(t,q(:,3),'--');
    
   %   figure(3)
 %  hold on 
 %  plot(t,q(:,1),'--');
 %  plot(t,q(:,7));
   
 %     figure(4)
 %  hold on 
 %  plot(t,q(:,2));
  % plot(t,q(:,8));
    
 %   plot(data(i).q(:,3),data(i).q(:,6));
 %  plot(data(i).q(:,1),data(i).q(:,2));
 %xlabel('Time: t(s)'); 
 %ylabel('Pitch angle: $\theta$(rad)', 'Interpreter','latex');
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




function [value,isterminal,direction] = end_front_stance(~,q)  %between back leg stance and flight stance     %act on the back_stance_motion
	    % back leg liftoff event
       
		value = l_0 - q(1) ;
		isterminal =  1;
		direction =   -1;
end





end