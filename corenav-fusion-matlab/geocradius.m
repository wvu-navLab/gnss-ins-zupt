function radius = geocradius(geoLat)

sinlam=sin(geoLat);
% hard coded for Earth WGS84
f= 1/298.257223560;
R=6378137;
radius  = sqrt(( R.^2 )./( 1 + (1/(( 1 - f ).^2) - 1).*sinlam.^2 ));
