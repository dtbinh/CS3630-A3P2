% demo08_squareWorldMCL
% Demo generic Monte Carlo Localization

path(path,'threedee') 

A = [0, .3, .5588, .3556
0, -.7, .5588, .3556
.5588, .6556, .5588, .3556
.5588, -1.0556, .5588, .3556
1.1176, .6556, .5588, .3556
1.1176, -1.0556, .5588, .3556
.9412, -.2, .5588, .3556
1.5, -.2, .5588, .3556];

map = SquareMap(A, 2.5);

%% Create a map
%A = [
%    -3,2,6,1
%    -3,-4,4,2
%    3,-4,1,6
%    ]
%map=SquareMap(A)

%% and a robot with noisy odometry
V=diag([0.01, 0.1*pi/180].^2)
veh=Differential(V)
veh.add_driver(DeterministicPath('SensorLog.txt'));

%% and then a sensor with noisy readings
W=0.05^2;
sensor = RangeSensor(veh,map, W,'SensorLog.txt')

%% define two covariances for random noise Q and L (hmmm!)
% For Q, use the uncertainly estimates from A2!
Q = diag([0.01,0.01,0.1*pi/180].^2);
L = diag(0.1); 

%% Finally, construct ParticleFilter
pf = GenericParticleFilter(veh, sensor, Q, L, 200);

%% and run for 1000 steps
pf.run(300,'nouniform','nosense');