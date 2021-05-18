
function [P_s_old,x_s_old] = smoothback(P_plus_old,P_minus_new,STM_old,x_plus_old,x_minus_new,x_s_new,P_s_new,STM_s_new)
    STM_s_new=STM_old-eye(15).*x_s_new;                
    AA=P_plus_old*(STM_s_new)'*pinv(P_minus_new); %old = k-1, new = k
    x_s_old=x_plus_old+AA*(x_s_new-x_minus_new);
    P_s_old=P_plus_old+AA*(P_s_new-P_minus_new)*AA';
end
