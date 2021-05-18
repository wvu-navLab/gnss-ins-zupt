for i=1:length(meanslip.mean)
[chi_UT_est,chi_UT_est_cov] = UTfun(meanslip.mean(i),meanslip.sigma(i));
rearVel3(i)=chi_UT_est;
rearVelCov(i)=chi_UT_est_cov;
countS(i)=i;
end

% figure;plot(rearVelCov)
% plot(rearVel3);hold on; plot(rearVel3-3*rearVelCov);hold on; plot(rearVel3+3*rearVelCov)
% plot(rearVel3);hold on; plot(rearVel3-3*rearVelCov);hold on; plot(rearVel3+3*rearVelCov)
% fill( [countS fliplr(countS)],  [rearVel3-3*rearVelCov fliplr(rearVel3+3*rearVelCov)], 'r','DisplayName','3\sigma Covariance Hull' );

figure;
fill( [countS(1:end) fliplr(countS(1:end))],  [rearVel3(1:end)-3*rearVelCov(1:end) fliplr(rearVel3(1:end)+3*rearVelCov(1:end))], 'r','DisplayName','3\sigma Covariance Hull' );
hold on;
plot(rearVel3,'b')
% mean([median(abs(velBackLeft)),median(abs(velBackRight)),median(abs(velFrontLeft)),median(abs(velFrontRight))])
