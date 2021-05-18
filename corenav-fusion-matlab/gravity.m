function g = gravity(lat,height)
%GRAVITY		Plumb-bob gravity calculation. 
%       
%	g = gravity(lat,height)
%
%   INPUTS
%       lat = latitude in radians
%       height = height above the ellipsoid in meters
%
%   OUTPUTS
%       g = gravity in meters/second
%

%  REFERENCES
%       Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION
%       TECHNOLOGY, Peter Peregrinus Ltd. on behalf of the Institution
%       of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 4-98
%	Copyright (c) 1998 by GPSoft LLC
%	All Rights Reserved.
%

   if nargin<2,error('insufficient number of input arguments'),end
   
   h = height;
   [Rm,Rp] = radicurv(lat);
   Ro = sqrt(Rp*Rm);   % mean earth radius of curvature
   g0 = 9.780318*( 1 + 5.3024e-3*(sin(lat))^2 - 5.9e-6*(sin(2*lat))^2 );
   if h >= 0,
      g = g0/( (1 + h/Ro)^2 );
   end
   if h < 0,
      g = g0*(1 + h/Ro);
   end,
   if h < -Ro,
      error(' Input altitude is less than -(earth radius) ')
   end,
   