h(1)=figure;
subplot(311),
hold on
plot(TimeIMU,insAtt(1,:)*180/pi,'-.r','DisplayName','estimation')
% hold on
% plot(TimeIMU,sig1(1,:)*180/pi,'-.b','DisplayName','Cov')
ylabel('\phi_E_s_t (deg)')
legend('show');

PitchErr2=x_err(2,:)*180/pi;%*180/pi;
subplot(312),
hold on
plot(TimeIMU,insAtt(2,:)*180/pi,'-.r','DisplayName','Model')
ylabel('\theta_E_s_t (deg)')
legend('show');
YawErr2=insAtt(3,:)*180/pi;%*180/pi;
subplot(313),
hold on
plot(TimeIMU,YawErr2,'-.r','DisplayName','Model')
ylabel('\psi_E_s_t (deg)')


VerrN2= x_err(4,:)';%-GeneratedEnuVelocity(:,2);
VerrE2= x_err(5,:)';%-GeneratedEnuVelocity(:,1);
VerrD2= x_err(6,:)';%+GeneratedEnuVelocity(:,3);

h(2)=figure;
subplot(311),
hold on
plot(TimeIMU,insVel(1,:),'-.r','DisplayName','Model')
ylabel('V_N E_s_t (m/s)')
legend('show');
hold off
subplot(312),
hold on
plot(TimeIMU,insVel(2,:),'-.r','DisplayName','Model')
ylabel('V_E E_s_t (m/s)')
hold off
legend('show');
subplot(313),
hold on
plot(TimeIMU,insVel(3,:),'-.r','DisplayName','Model')
ylabel('V_D E_s_t (m/s)')
legend('show');
hold off
xlabel('TimeIMU ')
llhEst=zeros(3,L);
llhTrue=zeros(3,L);
llhEst2=zeros(3,L);

% figure,
% subplot(311),
% hold on
% plot(TimeIMU,insLLH(1,:)*180/pi,'-.r','DisplayName','Model')
% ylabel('lat E_s_t ')
% legend('show');
% hold off
% subplot(312),
% plot(TimeIMU,insLLH(2,:)*180/pi,'-.r','DisplayName','Model')
% ylabel('lon E_s_t ')
% legend('show');
% hold off
% subplot(313),
% hold on
% plot(TimeIMU,insLLH(3,:),'-.r','DisplayName','Model')
% ylabel('h E_s_t ');
% legend('show');
% hold off
% xlabel('TimeIMU ')


h(3)=figure;
fill( [tTimu'-tTimu(1) fliplr(tTimu'-tTimu(1))],  [ENU_3P(1,:) fliplr(ENU3P(1,:))], 'r','DisplayName','3\sigma Covariance Hull' );
hold on
plot(tTimu-tTimu(1),ENU(1,:),'-b','DisplayName','Filter Estimation East'); 
if gpsResults()
hold on;
plot(tTodom-min(tTodom),ENUGPS(1,1:length(tTodom)),'-k','DisplayName','GPS Solution East');
end
% hold on;
% plot(tTodom-min(tTodom),Pos_x-Pos_x(1),'-.g','DisplayName','odom')
% hold on

% hold on
% patch(ENU_3P(1,:), ENU3P(1,:), [.8 .8 .1]);
ylabel('E (m)')
xlabel('time (sec)')
legend('show');

h(4)=figure;

fill( [tTimu'-tTimu(1) fliplr(tTimu'-tTimu(1))],  [ENU_3P(2,:) fliplr(ENU3P(2,:))], 'r','DisplayName','3\sigma Covariance Hull' );
hold on
plot(tTimu-tTimu(1),ENU(2,:),'-b','DisplayName','Filter Estimation North'); 
if gpsResults()
hold on;
plot(tTodom-min(tTodom),ENUGPS(2,1:length(tTodom)),'-k','DisplayName','GPS Solution North');
end
% plot(tTimu-tTimu(1),ENU(2,:),'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),ENUGPS(2,1:length(tTodom)),'-.r','DisplayName','true')
% hold on;
% % plot(tTodom-min(tTodom),Pos_y-Pos_y(1),'-.g','DisplayName','odom')
% hold on
% plot(tTimu-tTimu(1),ENU3P(2,:),'--k')
% hold on
% plot(tTimu-tTimu(1),ENU_3P(2,:),'--k')
ylabel('N (m)')
xlabel('time (sec)')
legend('show');

h(5)=figure;

fill( [tTimu'-tTimu(1) fliplr(tTimu'-tTimu(1))],  [ENU_3P(3,:) fliplr(ENU3P(3,:))], 'r','DisplayName','3\sigma Covariance Hull' );
hold on
plot(tTimu-tTimu(1),ENU(3,:),'-b','DisplayName','Filter Estimation Up'); 
if gpsResults()
hold on;
plot(tTodom-min(tTodom),ENUGPS(3,1:length(tTodom)),'-k','DisplayName','GPS Solution Up');
end
% plot(tTimu-tTimu(1),ENU(3,:)-ENU(3,1),'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),ENUGPS(3,1:length(tTodom)),'-.r','DisplayName','true')
% hold on
% plot(tTimu-tTimu(1),ENU3P(3,:),'--k')
% hold on
% plot(tTimu-tTimu(1),ENU_3P(3,:),'--k')
ylabel('U (m)')
xlabel('time (sec)')
legend('show');

% h(3)=figure;
% plot(tTimu-tTimu(1),ENU(1,:),'-.b','DisplayName','LLN'); 
% hold on;
% plot(tTodom-min(tTodom),ENUGPS(1,1:length(tTodom)),'-.r','DisplayName','true')
% % hold on;
% % plot(tTodom-min(tTodom),Pos_x-Pos_x(1),'-.g','DisplayName','odom')
% hold on
% plot(tTimu-tTimu(1),ENU3P(1,:),'--k')
% hold on
% plot(tTimu-tTimu(1),ENU_3P(1,:),'--k')
% ylabel('E (m)')
% xlabel('time (sec)')
% legend('show');
% 
% 
% h(4)=figure;
% plot(tTimu-tTimu(1),ENU(2,:),'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),ENUGPS(2,1:length(tTodom)),'-.r','DisplayName','true')
% hold on;
% % plot(tTodom-min(tTodom),Pos_y-Pos_y(1),'-.g','DisplayName','odom')
% hold on
% plot(tTimu-tTimu(1),ENU3P(2,:),'--k')
% hold on
% plot(tTimu-tTimu(1),ENU_3P(2,:),'--k')
% ylabel('N (m)')
% xlabel('time (sec)')
% legend('show');
% 
% h(5)=figure;
% plot(tTimu-tTimu(1),ENU(3,:)-ENU(3,1),'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),ENUGPS(3,1:length(tTodom)),'-.r','DisplayName','true')
% hold on
% plot(tTimu-tTimu(1),ENU3P(3,:),'--k')
% hold on
% plot(tTimu-tTimu(1),ENU_3P(3,:),'--k')
% ylabel('U (m)')
% xlabel('time (sec)')
% legend('show');

% figure;
% plot(tTimu-min(tTimu),insLLH(2,:)*180/pi,'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),llhGPS(2,2:end)*180/pi,'-.r','DisplayName','true')
% ylabel('Longitude')
% xlabel('time (sec)')
% legend('show');

% figure;
% plot(tTimu-min(tTimu),insLLH(1,:)*180/pi,'-.b','DisplayName','LLN'); 
% hold on; 
% plot(tTodom-min(tTodom),llhGPS(1,2:end)*180/pi,'-.r','DisplayName','true')
% ylabel('Latitude ')
% xlabel('time (sec)')
% legend('show');

h(6)=figure;
plot(ENU(1,:),ENU(2,:),'.--r','DisplayName','Filter Estimation')
hold on
% plot(ENU(1,1),ENU(2,1),'o','DisplayName','CoreNav Start')
hold on
% plot(ENU(1,end),ENU(2,end),'*','DisplayName','CoreNav End')
if gpsResults()
hold on
plot(ENUGPS(1,:),ENUGPS(2,:),'.-k','DisplayName','GPS Solution')
end
hold on
plot(ENUcorrected(1,:),ENUcorrected(2,:),'*g','DisplayName','Zero-Updates','MarkerSize',10)
plot(dposy,dposx,'-c','DisplayName','WheelOdometry+IMUHeading','LineWidth',2);

if odomUpdate
hold on
plot(ENUcorrected1(1,:),ENUcorrected1(2,:),'ob','DisplayName','Detected Slip','MarkerSize',4)
% hold on
% plot(ENUcorrected2(1,:),ENUcorrected2(2,:),'*c','DisplayName','Mahalanobis')
end
if gpsResults
hold on
% plot(ENUGPS(1),ENUGPS(2),'o','DisplayName','Truth Start')
% hold on

% plot(ENUGPS(1,end),ENUGPS(2,end),'*','DisplayName','Truth End')
end
xlabel('E(m) ')
ylabel('N(m) ')
legend('show');
if gpsResults()
h(7)=figure;
subplot(311),plot(tTimu-tTimu(1),gpsLongerXYZ(1,:)-xyz(1,:),'DisplayName','x_{err}')
ylabel('x (m)')
xlabel('time (sec)')
subplot(312),plot(tTimu-tTimu(1),gpsLongerXYZ(2,:)-xyz(2,:),'DisplayName','y_{err}')
ylabel('y (m)')
xlabel('time (sec)')
subplot(313),plot(tTimu-tTimu(1),gpsLongerXYZ(3,:)-xyz(3,:),'DisplayName','z_{err}')
ylabel('z (m)')
xlabel('time (sec)')
end
% % h(8)=figure;
% % hold off
% % 
% % loyolagray = 1/255*[150,150,150];
% % for ii=1:1500:length(ENU(1:71599)) 
% % d(ii) = pdist([ENU(1,ii),ENU(2,ii);ENU3P(1,ii),ENU3P(2,ii)],'euclidean');
% % b(ii) = pdist([ENU(2,ii),ENU(3,ii);ENU3P(2,ii),ENU3P(3,ii)],'euclidean');
% % coords(:,:,ii)=ellipse3D(d(ii),b(ii),ENU(1,ii),ENU(2,ii),ENU(3,ii),300,pi/2,0,insAtt(3,ii));
% % % plot3([coords]')
% % % ELLIPSE(rx,ry,x0,y0,z0,Nb, pitch,roll,yaw) adds an on the XY plane 
% % % ellipse with semimajor axis of rx, a semimajor axis of radius ry centered 
% % % at the point x0,y0,z0 and a pose in 3D space defined by roll, pitch, and 
% % % yaw angles 
% % % plot(x,y,'k')% xxx(kkk)=x;
% % hold on
% % plot3(coords(1,:,ii),coords(2,:,ii),coords(3,:,ii),'Color',loyolagray)
% % % legend
% % end

% % plot3(ENU(1,:),ENU(2,:),ENU(3,:),'.r','DisplayName','Filter Estimate')
% % if gpsResults
% % hold on
% % % plot3(ENU3P(1,:),ENU3P(2,:),ENU3P(3,:),'--k','DisplayName','Covariance Ellipse')
% % % hold on
% % % plot3(ENU_3P(1,:),ENU_3P(2,:),ENU_3P(3,:),'--k')
% % % hold on
% % plot3(ENUGPS(1,:),ENUGPS(2,:),ENUGPS(3,:),'.-k','DisplayName','GPS Solution')
% % end
% % grid on
% % xlabel('East (m)')
% % ylabel('North (m)')
% % zlabel('Up (m)')
% % view(-73,40)
% % hold off
% % legend({'Filter Estimate','GPS Solution', 'Covariance Ellipse'})

% [envHigh, envLow] = envelope(ENUGPS(1,:),16,'peak');
% envMean = (envHigh+envLow)/2;
% 
% plot(tTodom-min(tTodom),ENUGPS(1,1:length(tTodom)), ...
%      tTodom-min(tTodom),envHigh(1,1:length(tTodom)), ...
%      tTodom-min(tTodom),envMean(1,1:length(tTodom)), ...
%      tTodom-min(tTodom),envLow(1,1:length(tTodom)))
figure;plot3(ENU(1,:),ENU(2,:),ENU(3,:))
hold on;
plot3(ENUGPS(1,:),ENUGPS(2,:),ENUGPS(3,:))
grid on