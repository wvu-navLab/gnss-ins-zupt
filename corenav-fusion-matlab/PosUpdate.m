function [insLLHPlus,R_EPlus,r_eb_e] = PosUpdate(insLLHMinus,insVelocityPlus,insVelocityMinus,Ro,ecc,dt)
    R_N = Ro*(1-ecc^2)/(1-ecc^2*sin(insLLHMinus(1))^2)^(3/2);
    % Radius of Curvature in East-West Direction (eq. 2.106)
    R_E = Ro/(1-ecc^2*sin(insLLHMinus(1))^2)^(1/2);
 
    % height update
    insLLHPlus(3,:)=insLLHMinus(3) - dt/2*(insVelocityMinus(3)+insVelocityPlus(3));
    
    % latitude update
    insLLHPlus(1,:)=insLLHMinus(1)+ dt/2*(insVelocityMinus(1)/(R_N+insLLHMinus(3))+ insVelocityPlus(1)/(R_N+insLLHPlus(3)));
    
    % longitude update
    R_EPlus= Ro/(1-ecc^2*sin(insLLHPlus(1))^2)^(1/2);
    
    insLLHPlus(2,:)=insLLHMinus(2)+ dt/2*(insVelocityMinus(2)/((R_E+insLLHMinus(3))*cos(insLLHMinus(1))) ...
        + insVelocityPlus(2)/((R_EPlus+insLLHPlus(3))*cos(insLLHPlus(1))));
    
    cos_lat = cos(insLLHPlus(1));
    sin_lat = sin(insLLHPlus(1));
    cos_long = cos(insLLHPlus(2));
    sin_long = sin(insLLHPlus(2));
    r_eb_e = [((R_E + insLLHPlus(3)) * cos_lat * cos_long);...
          ((R_E + insLLHPlus(3)) * cos_lat * sin_long);...
          (((1 - ecc^2) * R_E + insLLHPlus(3)) * sin_lat)];
    
end