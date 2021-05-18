function xyz = llh2xyz(llh)
%LLH2XYZ  Convert from latitude, longitude and height
%         to ECEF cartesian coordinates.  WGS-84
%
%	xyz = LLH2XYZ(llh)	
%
%	llh(1) = latitude in radians
%	llh(2) = longitude in radians
%	llh(3) = height above ellipsoid in meters
%
%	xyz(1) = ECEF x-coordinate in meters
%	xyz(2) = ECEF y-coordinate in meters
%	xyz(3) = ECEF z-coordinate in meters

%	Reference: Understanding GPS: Principles and Applications,
%	           Elliott D. Kaplan, Editor, Artech House Publishers,
%	           Boston, 1996.
%
%	M. & S. Braasch 10-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.

	phi = llh(1);
	lambda = llh(2);
	h = llh(3);

	a = 6378137.0000;	% earth semimajor axis in meters
	b = 6356752.3142;	% earth semiminor axis in meters	
	e = sqrt (1-(b/a).^2);

	sinphi = sin(phi);
	cosphi = cos(phi);
	coslam = cos(lambda);
	sinlam = sin(lambda);
	tan2phi = (tan(phi))^2;
	tmp = 1 - e*e;
	tmpden = sqrt( 1 + tmp*tan2phi );

	x = (a*coslam)/tmpden + h*coslam*cosphi;

	y = (a*sinlam)/tmpden + h*sinlam*cosphi;

	tmp2 = sqrt(1 - e*e*sinphi*sinphi);
	z = (a*tmp*sinphi)/tmp2 + h*sinphi;

	xyz=[x;y;z];
