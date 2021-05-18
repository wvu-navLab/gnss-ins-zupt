odompostfit(:,odomUptCount)=postFitOdom.z-postFitOdom.H*x_err;
S=postFitOdom.H*P*postFitOdom.H'+[0.00045,0,0,0;0,0.1152,0,0;0,0,0.0025,0;0,0,0,0.0025];
chisq(:,odomUptCount)=odompostfit(:,odomUptCount)'*inv(S)*odompostfit(:,odomUptCount);
mahala(:,odomUptCount)=sqrt(odompostfit(:,odomUptCount)'*inv(S)*odompostfit(:,odomUptCount));
%                 if mahala(1,odomUptCount)>5

%                 if mahala(1,odomUptCount)>5
%
%                     insAtt(:,i)=insAtt_old;
%                     insVel(:,i)=insVel_old;
%                     insLLH(:,i)=insLLH_old;
%                     P=P_old;
%                     x_err=x_err_old;
%                     LLHcorrected2(:,cttr2)=insLLH(:,i);
%                     cttr2=cttr2+1;
%                 end
slipR(:,odomUptCount)=(rearVel(odomUptCount)-sqrt(insVel(1,i)^2+insVel(2,i)^2))/rearVel(odomUptCount);
if slipR(:,odomUptCount) < -50
    slipR (:,odomUptCount) = 0;
end
slipF(:,odomUptCount)=(frontVel(odomUptCount)-sqrt(insVel(1,i)^2+insVel(2,i)^2))/rearVel(odomUptCount);
if slipF(:,odomUptCount) < -50
    slipF (:,odomUptCount) = 0;
end
if odomUptCount > 25
    if abs(slipF(1,odomUptCount))>0.3
        
        LLHcorrected1(:,cttr0)=insLLH(:,i);
        dadas(cttr0)=std(mahala(1,(odomUptCount-25):odomUptCount));
        cttr0=cttr0+1;
        
        if mahala(1,odomUptCount)>5
            
            %         insAtt(:,i)=insAtt_old;
            %         insVel(:,i)=insVel_old;
            %         insLLH(:,i)=insLLH_old;
            %         P=P_old;
            %         x_err=x_err_old;
            LLHcorrected2(:,cttr2)=insLLH(:,i);
            cttr2=cttr2+1;
        end
        
    end
end