function [ellipseCoords, h] = ellipse3D( rx,ry,x0,y0,z0,Nb,pitch,roll,yaw,graph,C )
%% Definition
%
% Ellipse3D generates a three row, single column vector that holds the
% coordinates of an ellipse in 3D space and can optionally plot it.
%
% Generation of the ellipse coordinates occurs via the following
% methodology:
%
%   1. Plot an ellipse on the XY plane with semimajor axis of radius rx
%      along X axis and semimajor axis of radius ry along Y axis
%
%   2. Rotate ellipse CCW about X axis by 'pitch' radians. 0 leaves ellipse
%      on XY plane. pi/2 rotates CCW about positive X-axis and puts ellipse
%      on XZ plane
%
%   3. Rotate ellipse CCW about Y axis by 'roll' radians. 0 leaves
%      ellipse on XY plane. pi/2 rotates CCW about positive Y-axis and puts
%      ellipse on YZ plane
%
%   4. Rotate ellipse CCW about Z axis by 'yaw'
%      radians. 0 leaves ellipse on XY plane. pi/2 rotates CCW about
%      positive Z-axis and (leaving ellipse on XY plane)
%
%   5. Apply offsets of x0, y0, and z0 to ellipse
%
%   6. Optionally generate 3D plot of ellipse
%
%% Inputs
%
% rx - length of radius of semimajor axis of ellipse along x-axis
% ry - length of radius of semimajor axis of ellipse along y-axis
% x0 - origin of ellipse with respect to x-axis
% y0 - origin of ellipse with respect to y-axis
% z0 - origin of ellipse with respect to z-axis
% Nb - number of points used to define ellipse
% pitch - angle of pitch in radians of ellipse wRt +x-axis
% roll - angle of roll in radians of ellipse wRt +y-axis
% yaw - angle of yaw in radians of ellipse wRt +z-axis
% graph - flag that tells function whether or not to graph.
%   0 - do not graph
%   1 - graph and let this function define graphing options
%   >1 - graph, but allow other functions to define graphing options
% C - color of ellipse to be plotted. Acceptable input either in character
%   form ('r') or RGB form ([0 .5 1])
%
%% Outputs
%
% ellipseCoords - holds coordinates of ellipse in form:
% [ X
%   Y
%   Z ]
% h - graphics handle of ellipse if graphed, 0 otherwise
%% Usage Examples
% ELLIPSE(rx,ry,x0,y0,z0) adds an on the XY plane ellipse with semimajor
% axis of rx, a semimajor axis of radius ry centered at the point x0,y0,z0.
%
% ELLIPSE(rx,ry,x0,y0,z0,Nb), Nb specifies the number of points
% used to draw the ellipse. The default value is 300. Nb may be used
% for each ellipse individually.
%
% ELLIPSE(rx,ry,x0,y0,z0,Nb, pitch,roll,yaw) adds an on the XY plane 
% ellipse with semimajor axis of rx, a semimajor axis of radius ry centered
% at the point x0,y0,z0 and a pose in 3D space defined by roll, pitch, and
% yaw angles
%
% as a sample of how ellipse works, the following produces a red ellipse
% tipped up with a pitch of 45 degrees
% [coords, h] = ellipse3D(1,2,0,0,0,300,pi/4,0,0,1,'r');
%
% note that if rx=ry, ELLIPSE3D plots a circle
%
%% METADATA
% Author: William Martin
% Author Contact: wmartin8@utk.edu
% Institution: University of Tennessee, Knoxville
% Date Created: 6/22/2012
% Last Updated: 6/25/2012
% Inspiration:
% ellipse.m
%   written by D.G. Long, Brigham Young University, based on the
%   CIRCLES.m original
%   written by Peter Blattner, Institute of Microtechnology, University of
%   Neuchatel, Switzerland, blattner@imt.unine.ch
%% Parse Inputs
% Check the number of input arguments
if nargin<=1,error('Not enough arguments');end;
if nargin<1,rx=[];end;
if nargin<2,ry=[];end;
if nargin<3,x0=[];end;
if nargin<5,x0=[];y0=[];z0=[];end;
if nargin<6,Nb=[];end
if nargin<7,pitch=[];end
if nargin<8,roll=[];end
if nargin<9,yaw=[];end
if nargin<10,graph=[];end
if nargin<11,C=[];end
% set up the default values
if isempty(rx),rx=1;end;
if isempty(ry),ry=1;end;
if isempty(x0),x0=0;end;
if isempty(y0),y0=0;end;
if isempty(z0),z0=0;end;
if isempty(Nb),Nb=300;end;
if isempty(pitch),pitch=0;end;
if isempty(roll),roll=0;end;
if isempty(yaw),yaw=0;end;
if isempty(graph),graph=1;end;
if isempty(C),C='b';end;
h=0;
if ischar(C)
    C=C(:);
else
    C=[0,0,1]; % Make blue
end
% Ensure that all input values are scalars
if length(rx)>1,error('too many values for rx');end;
if length(ry)>1,error('too many values for ry');end;
if length(x0)>1,error('too many values for x0');end;
if length(y0)>1,error('too many values for y0');end;
if length(z0)>1,error('too many values for z0');end;
if length(Nb)>1,error('too many values for Nb');end;
if length(pitch)>1,error('too many values for pitch');end;
if length(roll)>1,error('too many values for roll');end;
if length(yaw)>1,error('too many values for yaw');end;
if length(graph)>1,error('too many values for graph');end;
%% Mathematical Formulation
% Declare angle vector theta (t in parametric equation of ellipse)
the=linspace(0,2*pi,Nb);
% Create X and Y vectors using parametric equation of ellipse
X=rx*cos(the);
Y=ry*sin(the);
% Declare the Z plane as all zeros before transformation
Z=zeros(1, length(X));
% Define rotation matrix about X axis. 0 leaves ellipse on XY plane. pi/2
% rotates CCW about X-axis and puts ellipse on XZ plane
Rpitch = [1          0           0          ;...
          0          cos(pitch)  -sin(pitch);...
          0          sin(pitch)  cos(pitch)];
% Define rotation matrix about Y axis. 0 leaves ellipse on XY plane. pi/2
% rotates CCW about Y-axis and puts ellipse on YZ plane
Rroll = [cos(roll)   0   sin(roll)  ;...
         0           1   0          ;...
         -sin(roll)  0   cos(roll)] ;
% Define rotation matrix about about Z axis. 0 leaves ellipse on XY plane. 
% pi/2 rotates CCW about Z-axis and (leaving ellipse on XY plane)
Ryaw = [cos(yaw)   -sin(yaw)  0;...
        sin(yaw)   cos(yaw)   0;...
        0           0         1];
% Apply transformation
for i=1:length(X)
    
    xyzMat = [X(i);Y(i);Z(i)]; % create temp values
    temp = Rpitch*xyzMat; % apply pitch
    temp = Rroll*temp; % apply roll
    temp = Ryaw*temp; % apply yaw
    X(i) = temp(1); % store results
    Y(i) = temp(2);
    Z(i) = temp(3);
    
end
% Apply offsets
X = X + x0;
Y = Y + y0;
Z = Z + z0;
%% Graphing
if graph > 0
    
    if graph == 0
        axis equal;
        axis auto;
        axis vis3d;
        grid on;
        view(3);
        
        axisLength = max([rx ry]) + max([abs(x0) abs(y0) abs(z0)]);
%         h3=line([0,axisLength],[0,0],[0,0]);
%         set(h3,'Color','b');
%         text(axisLength,0,0,'X');
        
%         h2=line([0,0],[0,axisLength],[0,0]);
%         set(h2,'Color','g');
%         text(0,axisLength,0,'Y');
%         
%         h1=line([0,0],[0,0],[0,axisLength]);
%         set(h1,'Color','r');
%         text(0,0,axisLength,'Z');
        
        h = line(X,Y,Z); % plot ellipse in 3D
        set(h,'color',C); % Set color of ellipse
    else
    
        h = line(X,Y,Z); % plot ellipse in 3D
        set(h,'color',C); % Set color of ellipse
    end
    
end
ellipseCoords = [X;Y;Z]; % define return values
end
