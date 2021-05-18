%% updates
odomUpdate=true;
zeroUpdate=true;
nonHolo=false;
backProp=true;
gpsResults=true;
gaussianProcess=true;
%% 
L=length(tTimu);
ba=zeros(3,L);
bg=zeros(3,L);
% ba(:,1)=[7.549135895545244e-04;0.001805886718248;0.002395586011438]; %95
ba(:,1)=[std(Ax(1:200));std(Ay(1:200));std(Az(1:200))];
% ba(:,1)=[-0.176671490035842;-0.0429240322667574;0.002395586011438];
% bg(:,1)=[8.674066138832567e-05;1.005391303275586e-04;8.919962404601004e-05]; %95
bg(:,1)=[std(Gx(1:200));std(Gy(1:200));std(Gz(1:200))];
% bg(:,1)=[-0.000190189015981112;0.000235157319569362;8.919962404601004e-05];
% ba(:,1)=[0;0;0];
% bg(:,1)=[0;0;0];
A=0.272; % meter  0.544/2 distance from IMU to Wheel
% initHead=atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1),sin(lon2-lon1)*cos(lat2));
% T_r=0.125; % rear track width m
% s_or=0;
% sigma_or_L=.03;%0.03
% sigma_or_R=.03;%0.03
% sigma_cmc=.05;
% s_delta_or=0;
T_r=0.685;%0.685; % rear track width m
s_or=-0.07;
sigma_or_L=.03;%0.03
sigma_or_R=.03;%0.03
sigma_cmc=.05;
s_delta_or=-0.6;

x_err=zeros(15,1);
transportRate=zeros(1,L);
lateralVelocity=zeros(1,L);
% P=blkdiag(((2*pi/180)^2)*(1/250),((2*pi/180)^2)*(1/250),(2*pi/180)^2,...  % att
%     eye(3,3)*((0.01)^2)*(1/250),...                                        % vel
%     eye(2,2)*(((10e-5)*pi/180)^2)*(1/250),1^2,...                          % pos
%     eye(3,3).*(ba(1:3)'.^2)*(1/250),...
%     eye(3,3).*(bg(1:3)'.^2)*(1/250));

% P=blkdiag(((1*pi/180)^2)*(1/250),((1*pi/180)^2)*(1/250),((2*pi/180)^2)*(1/250),...%*(1/250),...  % att
%     eye(3,3)*((0.01)^2)*(1/250),...                                        % vel
%     eye(2,2)*(((10e-5)*pi/180)^2)*(1/250),0.1^2,...                          % pos
%     eye(3).*ba(:,1),...
%     eye(3).*bg(:,1));

P=blkdiag(((1*pi/180)^2)*(1/250),((1*pi/180)^2)*(1/250),((2*pi/180)^2)*(1/250),...%*(1/250),...  % att
    eye(3,3)*((0.01)^2)*(1/250),...                                        % vel
    eye(3,3)*0,...                          % pos
    eye(3).*ba(:,1),...
    eye(3).*bg(:,1));
sig1(1)=3*sqrt(P(1,1));
sig2(1)=3*sqrt(P(2,2));
sig3(1)=3*sqrt(P(3,3));
sig4(1)=3*sqrt(P(4,4));
sig5(1)=3*sqrt(P(5,5));
sig6(1)=3*sqrt(P(6,6));
sig7(1)=3*sqrt(P(7,7));
sig8(1)=3*sqrt(P(8,8));
sig9(1)=3*sqrt(P(9,9));
dt_odom(:,1)=0.1;
x_State=zeros(15,L);
x_State(:,1)=x_err(:,1);
insAtt=zeros(3,L);
insVel=zeros(3,L);
insLLH=zeros(3,L);
% insLLHCorr=zeros(3,L);
omega_b_ib=zeros(1,3);
f_ib_b=zeros(1,3);

insAtt(:,1)=[0.0;0.0;yaw]*pi/180; %s1320,s5.313% s5:313%s3s4:134%-s1-2320 %shrt1:143
%t1:194.5,t8-t6-t4-t2:312.5 t9:151.5,t3:158.5 t5=167.5  t10:47 t11:238 t6=312.5
%t17:2.3 %t21:1 %t22:3 pebble76
insVel(:,1)=[0.0;0.0;0.0]';
insLLH(:,1)=[llhGPS(1,1),llhGPS(2,1),llhGPS(3,1)];
insLLHCorr(:,1)=insLLH(:,1);
insXYZ(:,1)=[gpsECEF.x(1);gpsECEF.y(1);gpsECEF.z(1)];
xyzInit=insXYZ(:,1);

TimeIMU=zeros(1,L);

alpha_b_ib=zeros(1,3);
v_ib_b=zeros(1,3);
omega_ie = 7.292115e-5; % rotation of Earth in rad/sec
Ro = 6378137; % WGS84 Equatorial Radius
Rp = 6356752.31425; % WGS84 Polar Radius
flat= 1/298.257223563; % WGS84 Earth flattening
ecc = 0.0818191909425; % WGS84 Eccentricity
TimeIMU(1)=0;
TimeODOM=(tTodom-tTodom(1));

H11=zeros(1,3);
H12=zeros(1,3);
H21=zeros(1,3);
H31=zeros(1,3);
H32=zeros(1,3);
H41=zeros(1,3);
H42=zeros(1,3);
H24=zeros(1,3);
z11=zeros(1,1);
z21=zeros(1,1);
z31=zeros(1,1);
z41=zeros(1,1);
kk=2;
k=1;
counter=2;
bb(1)=1;
dtTot=0;
stationary(1)=0;
statRate(1)=1;
leverArm=zeros(3,1);
cttr1=1;
cttr2=1;
cttr3=1;
cttr0=1;
offsetCtr=0;
aaa=1;
zeroUptCount=0;
odomUptCount=1;
slipCounter=0;
case1=false;%imu only
case2=true;%imu+predictionR
case3=false;%imu+predictionFixed
flag=true;
gpFlag=false;
stopFlag=false;
startRecording=0;
stopRecording=0;
utc=1;
