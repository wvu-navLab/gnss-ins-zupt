% wheel_radius=0.11;%meters
% wheelbase=0.555; %meters
% i=1;
% for i=2:min(length(rearLeftPos),length(lin_x))
% DeltaPosRearRightWheel=rearRightPos(i)-rearRightPos(i-1);
% DeltaPosRearLeftWheel=rearLeftPos(i)-rearLeftPos(i-1);
% 
% DeltaPosRearWheel(i)=(DeltaPosRearLeftWheel+DeltaPosRearRightWheel)/2;
% rearVel(i)=(rearRightVel(i)-rearLeftVel(i))*wheel_radius/2; %lin_x from odom
% 
% % DeltaPosFrontWheel(i)=(DeltaPosFrontLeftWheel+DeltaPosFrontRightWheel)/2;
% frontVel(i)=(frontRightVel(i)-frontLeftVel(i))*wheel_radius/2; %lin_x from odom
% 
% DeltaPosFrontRightWheel=frontRightPos(i)-frontRightPos(i-1);
% DeltaPosFrontLeftWheel=frontLeftPos(i)-frontLeftPos(i-1);
% DeltaPosRightWheel(i)=(DeltaPosFrontRightWheel+DeltaPosRearRightWheel)/2; %rad
% DeltaPosLeftWheel(i)=-(DeltaPosFrontLeftWheel+DeltaPosRearLeftWheel)/2; %rad
% heading(i)=(DeltaPosRightWheel(i)-DeltaPosLeftWheel(i))/(wheelbase); %ang_z from odom
% headRate(i)=(-rearLeftVel(i)-(rearRightVel(i)+frontRightVel(i))/2)*wheel_radius/(0.5);
% headRateF(i)=(frontLeftVel(i)-frontRightVel(i))/(0.272);
% velFrontLeft(i)=frontLeftVel(i)*wheel_radius;
% velFrontRight(i)=frontRightVel(i)*wheel_radius;
% velBackLeft(i)=rearLeftVel(i)*wheel_radius;
% velBackRight(i)=rearRightVel(i)*wheel_radius;
% end


wheel_radius=0.11;%meters
wheelbase=0.555; %meters
wheel_width=0.685;
i=1;
heading(1)=insAtt(3,1);
headingF(1)=insAtt(3,1);
bearing(1)=insAtt(3,1);
dposx(1)=0.0;
dposy(1)=0.0;

for i=2:min(length(rearLeftPos),length(lin_x))
delta_time_odom=tTodom(i)-tTodom(i-1);    
% DeltaPosRearRightWheel=rearRightPos(i)-rearRightPos(i-1);
% DeltaPosRearLeftWheel=rearLeftPos(i)-rearLeftPos(i-1);
% 
% DeltaPosRearWheel(i)=(DeltaPosRearLeftWheel+DeltaPosRearRightWheel)/2;
% rearVel(i)=(rearRightVel(i)-rearLeftVel(i))*wheel_radius/2; %lin_x from odom
% 
% % DeltaPosFrontWheel(i)=(DeltaPosFrontLeftWheel+DeltaPosFrontRightWheel)/2;
% frontVel(i)=(frontRightVel(i)-frontLeftVel(i))*wheel_radius/2; %lin_x from odom
% 
% DeltaPosFrontRightWheel=frontRightPos(i)-frontRightPos(i-1);
% DeltaPosFrontLeftWheel=frontLeftPos(i)-frontLeftPos(i-1);
% DeltaPosRightWheel(i)=(DeltaPosFrontRightWheel+DeltaPosRearRightWheel)/2; %rad
% DeltaPosLeftWheel(i)=-(DeltaPosFrontLeftWheel+DeltaPosRearLeftWheel)/2; %rad
% 
% heading(i)=(DeltaPosRightWheel(i)-DeltaPosLeftWheel(i))/(0.688); %ang_z from odom
% headRate(i)=(-rearLeftVel(i)-(rearRightVel(i)+frontRightVel(i))/2)*wheel_radius/(0.5);
% headRateF(i)=(frontLeftVel(i)-frontRightVel(i))/(0.272);

velFrontLeft(i)=-frontLeftVel(i)*wheel_radius;
velFrontRight(i)=frontRightVel(i)*wheel_radius;
velBackLeft(i)=-rearLeftVel(i)*wheel_radius;
velBackRight(i)=rearRightVel(i)*wheel_radius;

rearVel(i)=0.5*(velBackLeft(i)+velBackRight(i));
frontVel(i)=0.5*(velFrontLeft(i)+velFrontRight(i));
odomVel(i)=0.5*(rearVel(i)+frontVel(i)); 

headRate(i)= (velBackLeft(i)-velBackRight(i))/wheel_width;
headRateF(i)= (velFrontLeft(i)-velFrontRight(i))/wheel_width;

heading(i)=heading(i-1)+headRate(i)*delta_time_odom;
    if heading(i) > pi
        heading(i)= heading(i) - (floor(heading(i) / (2 * pi)) + 1) * 2 * pi;
    elseif heading(i)< -pi
        heading(i) = heading(i) + (floor(heading(i) / (-2 * pi)) + 1) * 2 * pi;
    end
    
headingF(i)=headingF(i-1)+headRateF(i)*delta_time_odom;  
    if headingF(i) > pi
        headingF(i)= headingF(i) - (floor(headingF(i) / (2 * pi)) + 1) * 2 * pi;
    elseif headingF(i)< -pi
        headingF(i) = headingF(i) + (floor(headingF(i) / (-2 * pi)) + 1) * 2 * pi;
    end
bearing(i)=bearing(i-1)-ang_z(i)*delta_time_odom;    
    if bearing(i) > pi
        bearing(i)= bearing(i) - (floor(bearing(i) / (2 * pi)) + 1) * 2 * pi;
    elseif bearing(i)< -pi
        bearing(i) = bearing(i) + (floor(bearing(i) / (-2 * pi)) + 1) * 2 * pi;
    end
    dposx(i)=dposx(i-1)+odomVel(i)*delta_time_odom*cos(bearing(i));
    dposy(i)=dposy(i-1)+odomVel(i)*delta_time_odom*sin(bearing(i));

end
%  figure;plot(dposy,dposx);
 
%  figure;plot(tTodom-tTodom(1),bearing)
%  hold on
%  plot(tTodom-tTodom(1),heading)
%  plot(tTodom-tTodom(1),headingF)
%  plot(tTimu-tTimu(1),insAtt(end,:))

