% testThreeDee: test ray-polygon intersection

%% Create a room
X=[-2 2 2 -2 -2];
Y=[-1 -1 1 1 -1];
[V,P] = wall(X,Y);

%% Generate normals and transforms
[VT,PT] = triangulate(V,P)
[N,d] = normals(VT,PT)
T = polygonT(VT,PT);

%% Shoot rays from the middle of the room to all 4 sides
O=[0;0;0];

[polygon,intersection,t] = raytrace(N,d,T,O,[0;1;0]);
CHECK('north',all(intersection==[0;1;0]));
CHECK('north',t==1);

[polygon,intersection,t] = raytrace(N,d,T,O,[1;0;0]);
CHECK('east',all(intersection==[2;0;0]));
CHECK('east',t==2);

[polygon,intersection,t] = raytrace(N,d,T,O,[0;-1;0]);
CHECK('south',all(intersection==[0;-1;0]));
CHECK('south',t==1);

[polygon,intersection,t] = raytrace(N,d,T,O,[-1;0;0]);
CHECK('west',all(intersection==[-2;0;0]));
CHECK('west',t==2);

