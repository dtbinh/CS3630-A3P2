function [N,d] = normals(V,P)
% normals: find normals to polygon list V,P
% [N,d] = normals(V,P)
% V - 3*nv matrix of vertices
% P - m*np matrix of polygons, where m>=3
% returns:
% N - normals
% d - fourth component of polygon plane equation

% create matrix of first, second and third vertices
V1 = V(:,P(1,:));
V2 = V(:,P(2,:));
V3 = V(:,P(3,:));

% do cross product
N = cross(V2-V1,V3-V1);

% normalize the normals :-)
N = N./(ones(3,1)*sqrt(sum(N.*N)));

% find rest of plane equation
d = -sum(V1.*N);
