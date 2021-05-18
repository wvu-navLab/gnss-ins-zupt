for ss=1:length(gpsECEF.x)
llhGPS(:,ss)=xyz2llh([gpsECEF.x(ss);gpsECEF.y(ss);gpsECEF.z(ss)]);
end
for ss=1:length(gpsECEF.x)
ENUGPS(:,ss)=xyz2enu([gpsECEF.x(ss);gpsECEF.y(ss);gpsECEF.z(ss)],[gpsECEF.x(1);gpsECEF.y(1);gpsECEF.z(1)]);
end
