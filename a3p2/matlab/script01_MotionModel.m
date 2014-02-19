% demo08_squareWorldMCL
% Demo generic Monte Carlo Localization

path(path,'threedee') 

A = [-1.5, .5, .5588, .3556
-1.5, -.5, .5588, .3556
-.9412, .8556, .5588, .3556
-.9412, -.8556, .5588, .3556
-.3824, .8556, .5588, .3556
-.3824, -.8556, .5588, .3556
-.5588, 0, .5588, .3556
0, 0, .5588, .3556];

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
veh.add_driver(DeterministicPath('NoSensorLog.txt'));

%% and then a sensor with noisy readings
W=0.05^2;
sensor = RangeSensor(veh,map, W,'NoSensorLog.txt')

%% define two covariances for random noise Q and L (hmmm!)
% For Q, use the uncertainly estimates from A2!
Q = diag([0.01,0.01,0.1*pi/180].^2);
L = diag(0.1); 

%% Finally, construct ParticleFilter
pf = GenericParticleFilter(veh, sensor, Q, L, 200);

%% and run for 1000 steps
pf.run(1000,'nouniform','nosense');