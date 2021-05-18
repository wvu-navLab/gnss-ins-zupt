        % z and H values are integrated for IMU only below. When the Odometry
        % update is available (tTimu(i)>=tTodom(kk)) integrated values will be
        % updated by odometry measurements, used in the filter and destroyed.
        Cn2bPlus=Cb2nPlus';
        omega_b_ie=Cn2bPlus*omega_n_ie;
        omega_b_ei=-omega_b_ie;
        omega_b_eb=omega_b_ei+omega_b_ib';
        %% Measurement Matrix -- integration part for INS -- eq 16.48
        H11= H11+ [1,0,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H12= H12+ [1,0,0]*Cn2bPlus*dtIMU;
        H21= H21+ sin(insAtt(2,i))*[0,cos(insAtt(1,i)),sin(insAtt(1,i))]*Cn2bPlus*dtIMU;
        H31= H31+ [0,1,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H32= H32+ [0,1,0]*Cn2bPlus*dtIMU;
        H41= H41+ [0,0,1]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H42= H42+ [0,0,1]*Cn2bPlus*dtIMU;
        %% Measurement Innovation -- integration part for INS -- eq 16.42
        z11=z11+[1,0,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z21=z21+cos(insAtt(2,i))*dtIMU;
        z31=z31+[0,1,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z41=z41+[0,0,1]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;