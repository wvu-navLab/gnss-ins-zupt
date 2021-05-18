function DCMnb = eulr2dcm(eul_vect)
%EULR2DCM       Euler angle vector to direction cosine
%               matrix conversion.
%       
%	DCMnb = eulr2dcm(eul_vect)
%
%   INPUTS
%       eul_vect(1) = roll angle in radians 
%
%       eul_vect(2) = pitch angle in radians 
%
%       eul_vect(3) = yaw angle in radians 
%
%   OUTPUTS
%       DCMnb = 3x3 direction cosine matrix providing the
%             transformation from the navigation frame (NED)
%             to the body frame 
%

%   REFERENCE:  Titterton, D. and J. Weston, STRAPDOWN
%               INERTIAL NAVIGATION TECHNOLOGY, Peter
%               Peregrinus Ltd. on behalf of the Institution
%               of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 12-97; 02-05
%	Copyright (c) 1997-2005 by GPSoft LLC
%	All Rights Reserved.

  if nargin<1,error('insufficient number of input arguments'),end

 
  phi = eul_vect(1); theta = eul_vect(2); psi = eul_vect(3);

  cpsi = cos(psi); spsi = sin(psi);
  cthe = cos(theta); sthe = sin(theta);
  cphi = cos(phi); sphi = sin(phi);

  C1 = [cpsi  spsi 0; ...
        -spsi cpsi 0; ...
         0     0   1];
  
  C2 = [cthe  0  -sthe; ...
          0   1     0 ; ...
        sthe  0   cthe];
  C3 = [1   0    0;   ...
        0  cphi sphi; ...
        0 -sphi cphi];  
  DCMnb = C3*C2*C1;
  
% sin_phi = sin(eul_vect(1));
% cos_phi = cos(eul_vect(1));
% sin_theta = sin(eul_vect(2));
% cos_theta = cos(eul_vect(2));
% sin_psi = sin(eul_vect(3));
% cos_psi = cos(eul_vect(3));
% 
% % Calculate coordinate transformation matrix using (2.22)
% DCMnb(1,1) = cos_theta * cos_psi;
% DCMnb(1,2) = cos_theta * sin_psi;
% DCMnb(1,3) = -sin_theta;
% DCMnb(2,1) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
% DCMnb(2,2) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
% DCMnb(2,3) = sin_phi * cos_theta;
% DCMnb(3,1) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
% DCMnb(3,2) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
% DCMnb(3,3) = cos_phi * cos_theta;