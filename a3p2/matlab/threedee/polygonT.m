function T = polygonT(V,P)
% polygonT: create transformation matrices that transform each point in the polygon
%           to a point in the unit triangle (0,0,1)-(1,0,1)-(0,1,1)
%           this is all in homogeneous coordinates !
%           points in 3D are projected orthogonally on the embedding plane
%           i.e. V1 will be mapped to T*V1 = (0,0,1)
%                V2                to T*V2 = (1,0,1)
%                V3                to T*V3 = (0,1,1)
% T = polygonT(V,P)
% V - 3*nv matrix of vertices
% P - m*np matrix of polygons, where m>=3
% returns:
% T - 3*4*np tensor with the transformation matrices

[d,np] = size(P);
if(d~=3),error('P must 3*np'); end

% create matrix of first, second and third vertices
V1 = V(:,P(1,:));
V2 = V(:,P(2,:));
V3 = V(:,P(3,:));

B1 = V2-V1;
B2 = V3-V1;
B3 = cross(B1,B2);

% create unit transformation from triangle to polygon
T44 = repmat(eye(4,4),[1 1 np]);
T44(1:3,1,:)=B1;
T44(1:3,2,:)=B2;
T44(1:3,3,:)=B3;
T44(1:3,4,:)=V1;

T = zeros(3,4,np);
for i=1:np
  invT44 = inv(T44(:,:,i)); % invert
  T(:,:,i) = invT44([1;2;4],:);
end

