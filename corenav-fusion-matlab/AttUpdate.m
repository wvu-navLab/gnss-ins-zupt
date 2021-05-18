   function [insAttitudePlus,Cb2nPlus,Cb2nMinus,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie] = AttUpdate(insAttMinus,omega_ie,insLLHMinus,omega_b_ib,ecc,Ro,insVelMinus,dt)
%     insAttitudeMinus;
%    CbnMinus = eulr2dcm(insAttitudeMinus)'; % notice the tranpose
      Cn2bMinus = eulr2dcm(insAttMinus); %outputs Nav2Body
      Cb2nMinus=Cn2bMinus';

%     eq 2.123
    omega_n_ie=[omega_ie*cos(insLLHMinus(1));
        0;
        (-1.0)*omega_ie*sin(insLLHMinus(1))];
    % get body-rate with respect to interial
%     omega_b_ib= alpha_b_ib/dt;
    % skew-symmetric matrix (eq. 5.8)s
    Omega_b_ib= [0 -omega_b_ib(3) omega_b_ib(2);
        omega_b_ib(3) 0 -omega_b_ib(1);
        -omega_b_ib(2) omega_b_ib(1) 0];
    
    % define the Earth's rotation vector represented in the local
    % navigation frame axes (eq. 5.41)
    Omega_n_ie = omega_ie * [ 0 sin(insLLHMinus(1)) 0;
        -sin(insLLHMinus(1)) 0 -cos(insLLHMinus(1));
        0 cos(insLLHMinus(1)) 0];

    % define the transport rate term
    % this is due to the face that the local-navigation frame
    % center moves with respect to the Earth
    % Eq (5.42)
    
    % Radius of Curvature for North-South Motion (eq. 2.105)
    R_N = Ro*(1-ecc^2)/(1-ecc^2*sin(insLLHMinus(1))^2)^(3/2);
    % Radius of Curvature in East-West Direction (eq. 2.106)
    R_E = Ro/(1-ecc^2*sin(insLLHMinus(1))^2)^(1/2);
    
    % rotation rate vector
    omega_n_en = [ insVelMinus(2)/(R_E+insLLHMinus(3));
        -insVelMinus(1)/(R_N+insLLHMinus(3));
        (-insVelMinus(2)*tan(insLLHMinus(1)))/(R_E+insLLHMinus(3))];
    % skew-symetric
    Omega_n_en = [0 -omega_n_en(3) omega_n_en(2);
        omega_n_en(3) 0 -omega_n_en(1);
        -omega_n_en(2) omega_n_en(1) 0];
    
    % eq. 2.48
    omega_n_in=omega_n_en + omega_n_ie;
    
    alpha=omega_b_ib*dt;
%     lateralVelocity=norm([GeneratedEnuVelocity(:,1) GeneratedEnuVelocity(:,2)]);
    % integrate considering body-rate, Earth-rate, and craft-rate
    Cbb=eye(3)+(sin(norm(alpha))/(norm(alpha)))*(skewsymm(alpha)) + ((1-cos(norm(alpha)))/(norm(alpha))^2)*(skewsymm(alpha)).^2;
 
    Cb2nPlus= Cb2nMinus*(eye(3)+Omega_b_ib*dt)-(Omega_n_ie+Omega_n_en)*Cb2nMinus*dt;
%     Cb2nPlus= Cb2nMinus*(Cbb)-(Omega_n_ie+Omega_n_en)*Cb2nMinus*dt;
%     
%     omega_b_ie=CbnPlus'*omega_n_ie;
%     omega_b_ei=-omega_b_ie;
%     omega_b_eb=[Gx(i);Gy(i);Gz(i)]+omega_b_ei;
    
    insAttitudePlus=dcm2eulr(Cb2nPlus); %inputs Body to Nav
    end
      
      