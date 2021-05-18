    function [insVelPlus] = VelUpdate(Cb2nMinus, Cb2nPlus, v_ib_b,insVelMinus,insLLHMinus,omega_ie,Ro,ecc,dt)
    % specific-force transformation (eq. 5.48)
    V_n_ib=1/2*(Cb2nMinus+Cb2nPlus)*v_ib_b';
        % Radius of Curvature for North-South Motion (eq. 2.105)
    R_N = Ro*(1-ecc^2)/(1-ecc^2*sin(insLLHMinus(1))^2)^(3/2);
    % Radius of Curvature in East-West Direction (eq. 2.106)
    R_E = Ro/(1-ecc^2*sin(insLLHMinus(1))^2)^(1/2);
        Omega_n_ie = omega_ie * [ 0 sin(insLLHMinus(1)) 0;
        -sin(insLLHMinus(1)) 0 -cos(insLLHMinus(1));
        0 cos(insLLHMinus(1)) 0];
        % rotation rate vector
    omega_n_en = [ insVelMinus(2)/(R_E+insLLHMinus(3));
        -insVelMinus(1)/(R_N+insLLHMinus(3));
        (-insVelMinus(2)*tan(insLLHMinus(1)))/(R_E+insLLHMinus(3))];
    % skew-symetric
        Omega_n_en = [0 -omega_n_en(3) omega_n_en(2);
        omega_n_en(3) 0 -omega_n_en(1);
        -omega_n_en(2) omega_n_en(1) 0];
    % velocity integration considering, body to indertial velocity,
    % Earth-rate, Craft-rate, Gravity
    insVelPlus = insVelMinus + V_n_ib + ( [0;0;gravity(insLLHMinus(1),insLLHMinus(3))] - (Omega_n_en + 2*Omega_n_ie)*insVelMinus)*dt;
    end