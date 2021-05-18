function S = skewsymm(vect)
%SKEWSYMM		Convert a vector into corresponding
%              skew symmetric matrix. 
%       
%	S = skewsymm(vect)
%
%   INPUTS
%       vect = 3 element vector
%          vect(1) = sigma_x;
%          vect(2) = sigma_y;
%          vect(3) = sigma_z;
%
%   OUTPUTS
%       S = [   0      -sigma_z  sigma_y;
%             sigma_z     0     -sigma_x;
%            -sigma_y  sigma_x    0     ]
%

%	M. & S. Braasch 1-98
%	Copyright (c) 1997-98 by GPSoft
%	All Rights Reserved.
%

if nargin<1,error('insufficient number of input arguments'),end

     sigma_x = vect(1);
     sigma_y = vect(2);
     sigma_z = vect(3);
%
S = [  0      -sigma_z  sigma_y;
     sigma_z     0     -sigma_x;
     -sigma_y  sigma_x    0     ];
  
  
