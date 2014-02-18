function [VT,PT] = triangulate(V,P)
% triangulate: make vertex-faces into a triangulated version
% precondition: faces are convex
% [VT,PT] = triangulate(V,P)

VT=V;
PT=[];

% for all polygons in original list P
[m,n] = size(P);
for j=1:n
  p = P(:,j); % original polygon
  v1 = p(1);   % first vertex
  for i=2:m-1
    pt = [v1;p(i);p(i+1)]; % new polygon
    PT = [PT pt];
  end
end
