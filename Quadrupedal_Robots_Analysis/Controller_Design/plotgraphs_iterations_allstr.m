
function plotgraphs_iterations_allstr

qq1 = allstr_another(250,30);
qq2 = allstr_no_u;




for  i = 1:30
hold on 
plot(qq1(i).time,qq1(i).state(:,3),qq2(i).time,qq2(i).state(:,3),'--');
legend('Measurements of pitch angle','Desired pitch angle trajectory')
xlabel('Time : t(s)')
ylabel('Pitch rate : $\theta$(rad)', 'Interpreter','latex')
title('The degree of polynomial M is chosen to be 7.(Kp = 250, Kd = 30)')

%figure(2),plot(qq1(i).time,qq1(i).state(:,6),qq2(i).time,qq2(i).state(:,6),'--');
%legend('Desired pitch rate trajectory','Measurements of pitch rate')
%xlabel('Time : t(s)')
%ylabel('Pitch rate : $\dot{\theta}$(rad/s)', 'Interpreter','latex')
%title('The degree of polynomial M is chosen to be 17.(Kp = 250, Kd = 30)')
%figure(2),plot(qq1(i).time,qq1(i).state(:,6),qq2(i).time,qq2(i).state(:,6),':');
%legend('Desired pitch rate trajectory','Measurements of pitch rate')
%xlabel('Time : t(s)')
%ylabel('Pitch rate : $\dot{\theta}$(rad/s)', 'Interpreter','latex')

end
hold off
%figure(2),plot(time1,thetadot1,'--',time2,thetadot2,'-.');
%legend('Desired pitch rate trajectory','Measurements of pitch rate')
%xlabel('Time : t(s)')
%ylabel('Pitch rate : $\dot{\theta}$(rad/s)', 'Interpreter','latex')
%figure(3),plot(time1,x1,'--',time2,x2,'-.');
%legend('Desired horizontal position of CoM trajectory','Measurements of horizontal position')
%xlabel('Time : t(s)')
%ylabel('Horizontal position : x(m)', 'Interpreter','latex')
%figure(4),plot(time1,y1,'--',time2,y2,'-.');
%legend('Desired vertical position of CoM trajectory','Measurements of vertical position')
%xlabel('Time : t(s)')
%ylabel('Vertical position : y(m)', 'Interpreter','latex')
%figure(5),plot(time1,xdot1,'--',time2,xdot2,'-.');
%legend('Desired forward speed trajectory','Measurements of forward speed')
%xlabel('Time : t(s)')
%ylabel('Forward speed : $\dot{x}$(m/s)', 'Interpreter','latex')
%figure(6),plot(time1,ydot1,'--',time2,ydot2,'-.');
%legend('Desired vertical velocity trajectory','Measurements of vertical velocity')
%xlabel('Time : t(s)')
%ylabel('Vertical velocity : $\dot{y}$(m/s)', 'Interpreter','latex')
end