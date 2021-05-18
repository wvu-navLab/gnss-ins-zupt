timeGS=0:0.1:331.6;
figure;plot(diff(ENUGPS(1,:))/0.1,timeGS)
figure;plot(timeGS,diff(ENUGPS(1,:))/0.1)
figure;plot(timeGS,diff(ENUGPS(2,:))/0.1)
figure;plot(timeGS,diff(ENUGPS(3,:))/0.1)

figure;plot(gpsECEF.time(1:end-1)-gpsECEF.time(1),sqrt((diff(ENUGPS(1,:))/0.1).^2+(diff(ENUGPS(2,:))/0.1).^2))

figure;plot(tTodom(1:end-1)-tTodom(1),diff(dposy)/0.1)
figure;plot(tTodom(1:end-1)-tTodom(1),diff(dposx)/0.1)

WIOvelx=diff(dposx)/0.1;
WIOvely=diff(dposy)/0.1;
WOvelx=lin_x;
WOvely=lin_y;
GPSVelx=diff(ENUGPS(1,:))/0.1;
slipTruth=(WIOvelx-(GPSVelx))./WIOvelx;

% slipBL(:,odomUptCount)=((velBackLeft(odomUptCount))-(v(odomUptCount)))/(velBackLeft(odomUptCount));
% wheel_radius=0.11;%meters
% wheelbase=0.555; %meters
% wheel_width=0.685;
% velFrontLeft=-frontLeftVel*wheel_radius;
% velFrontRight=frontRightVel*wheel_radius;
% velBackLeft=-rearLeftVel*wheel_radius;
% velBackRight=rearRightVel*wheel_radius;
% rearVel=0.5*(velBackLeft+velBackRight);
% frontVel=0.5*(velFrontLeft+velFrontRight);
odomVel=0.5*(rearVel+frontVel); 
GPSVelx=smoothdata(diff(ENUGPS(1,1:(length(WIOvelx)+1)))/0.1,'gaussian',10);
GPSVely=diff(ENUGPS(2,1:(length(WIOvely)+1)))/0.1;
