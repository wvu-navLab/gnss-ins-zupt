figure;
subplot(4,3,1)
plot(slipMan(:,end));
xlim([0,205])
legend('exp1')
hold on;
subplot(4,3,4)
plot(slipAuto1(:,end));
xlim([0,205])
legend('exp2')
hold on; 
subplot(4,3,7)
plot(slipAuto2(:,end));
xlim([0,205])
legend('exp3')
hold on;
subplot(4,3,10)
plot(slipAuto3(:,end));
xlim([0,205])
legend('exp4')

subplot(4,3,2)
plot(linx1(:,end));
xlim([0,205])
ylim([0.5,0.8])
legend('exp1')
hold on;
subplot(4,3,5)
plot(linx2(:,end));
xlim([0,205])
ylim([0.5,0.8])
legend('exp2')
hold on; 
subplot(4,3,8)
plot(linx3(:,end));
xlim([0,205])
ylim([0.5,0.8])
legend('exp3')
hold on;
subplot(4,3,11)
plot(linx4(:,end));
xlim([0,205])
ylim([0.56,0.8])
legend('exp4')


subplot(4,3,3)
plot(insVEL1(:,end));
xlim([0,5000])
ylim([0.5,0.8])
legend('exp1')
hold on;
subplot(4,3,6)
plot(insVEL2(:,end));
xlim([0,5000])
ylim([0.5,0.8])
legend('exp2')
hold on; 
subplot(4,3,9)
plot(insVEL3(:,end));
xlim([0,5000])
ylim([0.5,0.8])
legend('exp3')
hold on;
subplot(4,3,12)
plot(insVEL4(:,end));
xlim([0,5000])
ylim([0.56,0.8])
legend('exp4')