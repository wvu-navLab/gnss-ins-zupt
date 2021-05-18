function [Rm,Rp,R_es_e] = radicurv(lat)
%RADICURV		Calculate meridian and prime radii of curvature. 
%       
%	[Rm,Rp] = radicurv(lat)
%
%   INPUTS
%       lat = geodetic latitude in radians
%
%   OUTPUTS
%       Rm = meridian radius of curvature in meters
%       Rp = prime radius of curvature in meters
%
%   NOTE
%       In some literature Rp is also referred to as the 'normal'
%       or 'transverse' radius of curvature
%
%   REFERENCES
%       Brockstein, A. and J. Kouba, "Derivation of Free Inertial, General
%       Wander Azimuth Mechanization Equations," Litton Systems, Inc., 
%       Guidance and Control Systems Division, Woodland Hills, California,
%       June 1969, Revised June 1981.
%
%       Kayton, M. and W. Fried, AVIONICS NAVIGATION SYSTEMS, 2nd edition,
%       John Wiley & Sons, New York, 1997.
%
%       Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION
%       TECHNOLOGY, Peter Peregrinus Ltd. on behalf of the Institution
%       of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 3-98
%	Copyright (c) 1997-98 by GPSoft LLC
%	All Rights Reserved.
%

if nargin<1,error('insufficient number of input arguments'),end

EQUA_RADIUS = 6378137.0;    % equatorial radius of the earth; WGS-84
ECCENTRICITY = 0.0818191908426;  % eccentricity of the earth ellipsoid

e2 = ECCENTRICITY^2;
den = 1-e2*(sin(lat))^2;
Rm = (EQUA_RADIUS*(1-e2))/( (den)^(3/2) );
Rp = EQUA_RADIUS/( sqrt(den) );


Top = EQUA_RADIUS*(sqrt(((1-e2)^2)+((sin(lat))^2)+((cos(lat))^2)));
Bottom = sqrt((1-e2*((sin(lat))^2)));
R_es_e = Top/Bottom;
