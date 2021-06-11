function [insVel_new,insAtt_new,insLLH_new,x_err_new,P_new] =gtsamUpt (insVel_old,insAtt_old,insLLH_old,x_err_old,P_old, llhGTSAM,Ro,ecc)
        S_p = [1, 0, 0;
                    0, 1, 0; 
                    0, 0, 1];
%         R_gtsam =1*[0.0001^2,0,0;
%                        0,0.0001^2,0;
%                        0,0,1^2];
        R_gtsam =[((20e-5)*pi/180)^2,0,0;
                       0,((20e-5)*pi/180)^2,0;
                       0,0,5^2];              
                   
      H_gtsam =[zeros(3), zeros(3), -eye(3),zeros(3),zeros(3)];             
    Cn2b_gtsam=eulr2dcm(insAtt_old);
    
    R_N = Ro*(1-ecc^2)/(1-ecc^2*sin(insLLH_old(1))^2)^(3/2);
    R_E = Ro/(1-ecc^2*sin(insLLH_old(1))^2)^(1/2);
    
    T_rn_p=[1.0/(R_N+insAtt_old(3)),0,0;
            0,1.0/((R_E+insAtt_old(3))*cos(insAtt_old(1))),0;
            0,0,-1];
        
 z_gtsam=S_p*(llhGTSAM-insLLH_old-T_rn_p*Cn2b_gtsam'*([0,0,0.272]'));
 K_gtsam = P_old * H_gtsam' * inv(H_gtsam * P_old * H_gtsam' + R_gtsam);
 x_err_new=x_err_old+K_gtsam*(z_gtsam-H_gtsam*x_err_old);
 
    insAtt_new=dcm2eulr((eye(3)-skewsymm(x_err_new(1:3)))*Cn2b_gtsam');
    insVel_new=insVel_old-x_err_new(4:6);
    insLLH_new=insLLH_old-x_err_new(7:9);
    x_err_new(1:9)=zeros(1,9);
    P_new=(eye(15) - K_gtsam*H_gtsam)*P_old*(eye(15)-K_gtsam*H_gtsam)' + K_gtsam*R_gtsam*K_gtsam';
end


        
                 