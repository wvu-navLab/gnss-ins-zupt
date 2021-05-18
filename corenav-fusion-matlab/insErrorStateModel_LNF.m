%R_E should be R_EPlus
%Cbn should be CbnPlus
function [STM] = insErrorStateModel_LNF(R_E,R_N,insLLH,insVel,dt,Cb2nPlus,omega_ie,omega_n_in,f_ib_b)

Z=zeros(3);
I=eye(3);

rGeoCent=geocradius(geocentricLattitude(insLLH(1)*180/pi));
g0=9.780318*( 1 + 5.3024e-3*(sin(insLLH(1)))^2 - 5.9e-6*(sin(2*insLLH(1)))^2 );

%% Eq:14.65
F12_1=[0, -1.0/(R_E+insLLH(3)), 0]; % Checked
F12_2=[1.0/(R_N+insLLH(3)), 0, 0]; % Checked
F12_3=[0, tan(insLLH(1))/(R_E+insLLH(3)), 0]; %Checked
%% Eq:14.66
F13_1=[omega_ie*sin(insLLH(1)), 0, insVel(2)/(R_E+insLLH(3))^2]; %Checked
F13_2=[0, 0, -insVel(1)/(R_N+insLLH(3))^2]; %Checked
F13_3=[omega_ie*cos(insLLH(1))+insVel(2)/((R_E+insLLH(3))*cos(insLLH(1))^2), 0, -insVel(2)*tan(insLLH(1))/(R_E+insLLH(3))^2]; %Checked
%% Eq:14.68
F22_1=[insVel(3)/(R_N+insLLH(3)), -((2.0)*insVel(2)*tan(insLLH(1))/(R_E+insLLH(3)))-(2.0)*omega_ie*sin(insLLH(1)), insVel(1)/(R_N+insLLH(3))]; %Checked
F22_2=[(insVel(2)*tan(insLLH(1))/(R_E+insLLH(3)))+(2.0)*omega_ie*sin(insLLH(1)), (insVel(1)*tan(insLLH(1))+insVel(3))/(R_E+insLLH(3)), (insVel(2)/(R_E+insLLH(3)))+(2.0)*omega_ie*cos(insLLH(1))]; %Checked
F22_3=[(-2.0)*insVel(1)/(R_N+insLLH(3)), (-2.0)*(insVel(2)/(R_E+insLLH(3)))-(2.0)*omega_ie*cos(insLLH(1)), 0]; %Checked
%% Eq:14.69
F23_1=[-(((insVel(2)^2)*sec(insLLH(1))^2)/(R_E+insLLH(3)))-(2*insVel(2)*omega_ie*cos(insLLH(1))), 0, (((insVel(2)^2)*(tan(insLLH(1))))/((R_E+insLLH(3))^2))-((insVel(1)*insVel(3))/(R_N+insLLH(3))^2)]; %Checked
F23_2=[((insVel(1)*insVel(2)*sec(insLLH(1))^2)/(R_E+insLLH(3)))+2*insVel(1)*omega_ie*cos(insLLH(1))-2*insVel(3)*omega_ie*sin(insLLH(1)), 0, -((insVel(1)*insVel(2)*tan(insLLH(1))+insLLH(2)*insLLH(3))/(R_E+insLLH(3))^2)]; %Checked
F23_3=[(2.0)*insVel(2)*omega_ie*sin(insLLH(1)), 0, (((insVel(2)^2)/(R_E+insLLH(3))^2)+((insVel(1)^2)/(R_N+insLLH(3))^2)-((2*g0)/(rGeoCent)))]; %Checked
%% Eq:14.70
F32_1=[(1.0)/(R_N+insLLH(3)), 0, 0]; %Checked
F32_2=[0, (1.0)/((R_E+insLLH(3))*cos(insLLH(1))), 0]; %Checked
F32_3=[0, 0, -1.0];%Checked
%% Eq:14.71
F33_1=[0, 0, -insVel(1)/(R_N+insLLH(3))^2]; %Checked
F33_2=[(insVel(2)*sin(insLLH(1)))/((R_E+insLLH(3))*cos(insLLH(1))^2), 0, -(insVel(2))/(((R_E+insLLH(3))^2)*cos(insLLH(1)))]; %Checked
F33_3=[0, 0, 0]; %Checked

F11= -skewsymm(omega_n_in); %Omega_n_in %Eq:14.64
F12= [F12_1; F12_2; F12_3];
F13= [F13_1; F13_2; F13_3];
F21= -skewsymm(Cb2nPlus*(f_ib_b)'); %Eq:14.67
F22= [F22_1; F22_2; F22_3];
F23= [F23_1; F23_2; F23_3];
F32= [F32_1; F32_2; F32_3];
F33= [F33_1; F33_2; F33_3];

PHI11 = I+F11*dt;
PHI12 = F12*dt;
PHI13 = F13*dt;
PHI15 = Cb2nPlus*dt;

PHI21 = F21*dt;
PHI22 = I+F22*dt;
PHI23 = F23*dt;
PHI24 = Cb2nPlus*dt;

PHI32 = F32*dt;
PHI33 = I+F33*dt;

%Eq:14.72
STM = [PHI11 PHI12 PHI13 Z PHI15;
       PHI21 PHI22 PHI23 PHI24 Z;
       Z PHI32 PHI33 Z Z;
       Z Z Z I Z;
       Z Z Z Z I];
   
end
