hold off

loyolagray = 1/255*[100,100,100];
for ii=1:750:length(ENU(1:100000)) 
d(ii) = pdist([ENU(1,ii),ENU(2,ii);ENU3P(1,ii),ENU3P(2,ii)],'euclidean');
b(ii) = pdist([ENU(2,ii),ENU(3,ii);ENU3P(2,ii),ENU3P(3,ii)],'euclidean');
coords(:,:,ii)=ellipse3D(d(ii),b(ii),ENU(1,ii),ENU(2,ii),ENU(3,ii),300,pi/2,0,insAtt(3,ii));
% plot3([coords]')
% ELLIPSE(rx,ry,x0,y0,z0,Nb, pitch,roll,yaw) adds an on the XY plane 
% ellipse with semimajor axis of rx, a semimajor axis of radius ry centered 
% at the point x0,y0,z0 and a pose in 3D space defined by roll, pitch, and 
% yaw angles 
% plot(x,y,'k')% xxx(kkk)=x;
hold on
plot3(coords(1,:,ii),coords(2,:,ii),coords(3,:,ii),'Color',loyolagray)

end

plot3(ENU(1,:),ENU(2,:),ENU(3,:),'.r')
hold on
plot3(ENU3P(1,:),ENU3P(2,:),ENU3P(3,:),'-k')
hold on
plot3(ENU_3P(1,:),ENU_3P(2,:),ENU_3P(3,:),'-k')
hold on
plot3(ENUGPS(1,:),ENUGPS(2,:),ENUGPS(3,:),'.-b')
grid on
view(-73,40)
hold off
