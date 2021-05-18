% clear
% load data/test24
% load data/gps/test24
figure;plot(gpsECEF.y-gpsECEF.y(1));hold on; plot(lin_x)
foo=0;
lin_x(1)=0;
for i=2:length(lin_x)
if (lin_x(i)-lin_x(i-1))>0 
    foo=i;
    break
end
end
odomCut
% clear ss prompt i foo a b 
% save test24run
