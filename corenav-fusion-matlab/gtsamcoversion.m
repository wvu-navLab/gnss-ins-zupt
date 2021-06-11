for ss=1:length(gtsam.X)
llhgtsam(:,ss)=xyz2llh([gtsam.X(ss);gtsam.Y(ss);gtsam.Z(ss)]);
end
for ss=1:length(gtsam.X)
ENUGTSAM(:,ss)=xyz2enu([gtsam.X(ss);gtsam.Y(ss);gtsam.Z(ss)],[gpsECEF.x(1);gpsECEF.y(1);gpsECEF.z(1)]);
ENUGTSAM2(:,ss)=xyz2enu([gtsam.X(ss);gtsam.Y(ss);gtsam.Z(ss)],[gtsam.X(1);gtsam.Y(1);gtsam.Z(1)]);
end
