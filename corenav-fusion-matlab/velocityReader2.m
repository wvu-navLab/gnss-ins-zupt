T = readtable('t10pos_stat.txt');
y=T(strcmp(T.x_POS, '$VELACC'),:);

y1=y(1:end/2,1:16);

y2=y(end/2:end,1:16);
y2sort=sortrows(y2,3);

TimeVely2=y2sort.time-y2sort.time(1);

figure;plot(TimeVely2,-y2sort.x)
figure;plot(TimeVely2,-y2sort.y)
figure;plot(TimeVely2,-y2sort.z)

TimeVely1=y1.time-y1.time(1);
figure;plot(TimeVely1,y1.x)
figure;plot(TimeVely1,y1.y)
figure;plot(TimeVely1,y1.z)
y2sort2=y2sort;
TimeVely22=y2sort2.time-y2sort2.time(1);
figure;plot(TimeVely22,-y2sort2.x)

figure;plot(TimeVely22,sqrt(y2sort2.x.^2+y2sort2.y.^2+y2sort2.z.^2))

figure;plot(tTimu-tTimu(1),sqrt(insVel(1,:).^2+insVel(2,:).^2+insVel(3,:).^2))
figure;plot(gpsECEF.time(1:end-1)-gpsECEF.time(1),sqrt((diff(ENUGPS(1,:))/0.1).^2+(diff(ENUGPS(2,:))/0.1).^2+(diff(ENUGPS(3,:))/0.1).^2))
aa=sqrt((diff(ENUGPS(1,:))/0.1).^2+(diff(ENUGPS(2,:))/0.1).^2+(diff(ENUGPS(3,:))/0.1).^2);
aab=smoothdata(aa,'gaussian',10);
figure;plot(gpsECEF.time(1:end-1)-gpsECEF.time(1),aab);