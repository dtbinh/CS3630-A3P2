function [V,P] = wall(X,Y,h,z)
% wall: Create a wall vertex and polygon list
% [V,P] = wall(X,Y,h,z)
%   X,Y - polyline that forms the base of the wall
%   h   - height of the wall (default 1)
%   z   - base of the wall (default 0)
% returns:
%   V   - vertex list
%   P   - polygon list

% process arguments
error(nargchk(2,4,nargin))
if(nargin<3), h=1; end
if(nargin<4), z=0; end
[m,n] = size(X);
if any([m n] ~= size(Y)), error('X and Y must be of same size'), end
if m~=1 & n~=1, error('X, Y must be vectors'), end
n = m*n;

% get base and top of wall
z1 = z;
z2 = z+h;

% create vertices
V1 = [reshape(X,n,1),reshape(Y,n,1),z1(ones(n,1))];
V2 = [reshape(X,n,1),reshape(Y,n,1),z2(ones(n,1))];
V = [V1;V2]';

% create faces
P = [1:n-1; 2:n; n+2:2*n; n+1:2*n-1];
