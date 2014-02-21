%RangeSensor Rangesensor class
%
% A concrete subclass of the Sensor class that implements a range
% sensor that provides robot-centric measurements of point features in 
% the world. To enable this it has references to a map of the world (Map object)
% and a robot moving through the world (Vehicle object).
%
% Methods::
%
% reading   range observation of random feature
% h         range observation of specific feature
% Hx        Jacobian matrix dh/dxv 
% Hxf       Jacobian matrix dh/dxf 
% Hw        Jacobian matrix dh/dw
%
% g         feature positin given vehicle pose and observation
% Gx        Jacobian matrix dg/dxv 
% Gz        Jacobian matrix dg/dz
%
% Properties (read/write)::
% W            measurement covariance matrix (2x2)
% interval     valid measurements returned every interval'th call to reading()
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also Sensor, Vehicle, Map, EKF.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

classdef RangeSensor < Sensor

    properties
        W           % measurment covariance
        r_range     % range limits

        randstream  % random stream just for Sensors
        log
        nrLines
        
        
    end

    properties (SetAccess = private)
        count       % number of reading()s
    end

    methods

        function s = RangeSensor(robot, map, W, filename, varargin)
            %RangeSensor.RangeSensor Range and bearing sensor constructor
            %
            % S = RangeSensor(VEHICLE, MAP, W, FILENAME, OPTIONS) is an object representing
            % a range and bearing angle sensor mounted on the Vehicle object
            % VEHICLE and observing an environment of known landmarks represented by the
            % map object MAP.  The sensor covariance is R (2x2) representing range and bearing
            % covariance.
            %
            % Options::
            % 'range', xmax            maximum range of sensor
            % 'range', [xmin xmax]     minimum and maximum range of sensor
            % 'skip', I                return a valid reading on every I'th
            %                          call
            % 'fail', [TMIN TMAX]      sensor simulates failure between 
            %                          timesteps TMIN and TMAX
            %
            % See also Sensor, Vehicle, Map, EKF.


            % call the superclass constructor
            s = s@Sensor(robot, map, varargin{:});

            s.randstream = RandStream.create('mt19937ar');

            opt.range = [];
            opt.thrange = [];

            [opt,args] = tb_optparse(opt, varargin);

            s.W = W;
            if ~isempty(opt.range)
                if length(opt.range) == 1
                    s.r_range = [0 opt.range];
                elseif length(opt.thrange) == 2
                    s.r_range = opt.range;
                end
            end
            
            s.log = dlmread(filename,' ');
            s.nrLines = size(s.log,1);
            
            if(isempty(s.log))
              fprintf('No data loaded! Simulating instead')
            end
            % Counting function
            s.count = 1;
        end

        function k = selectFeature(s)
            k = s.randstream.randi(s.map.nfeatures);
        end

        function [z,jf] = reading(s)
            %RangeSensor.h Landmark range and bearing
            %
            % [Z,K] = S.reading() is an observation of a random landmark where
            % Z=[R,THETA] is the range and bearing with additive Gaussian noise
            % of covariance R (specified to the constructor). K is the index of 
            % the map feature that was observed. If no valid measurement, ie. no
            % features within range, interval subsampling enabled or simulated 
            % failure the return is Z=[] and K=NaN.
            %
            % See also RangeSensor.h.
            
            % model a sensor that emits readings every interval samples
            if(~isempty(s.log))
              if s.count > s.nrLines
                  z = 999999; % Pseudo-Inf range
                  jf = 1;
                  s.count = s.count + 1;
                  return
              else
                  % Sensor values taken from the Fluke IR [0, 6400]
                  z = s.log(s.count,1); % Transform to distance
                  
                  % Assume power level of 140
                  % Obtain distance based on rough linear model
                  % Bound between effective range
                  if(z > 0)
                    z =  (-3.7468e-05)*z + 0.4847;
                  else
                    z = 999999;
                  end
                  jf = 1;
                  s.count = s.count + 1;
                  return
              end
            else
            
              s.count = s.count + 1;

              % check conditions for NOT returning a value
              z = [];
              jf = NaN;
              % sample interval
              if mod(s.count, s.interval) ~= 0
                  return;
              end
              % simulated failure
              if ~isempty(s.fail) && (s.count >= s.fail(1)) && (s.count <= s.fail(2))
                  return;
              end

              if ~isempty(s.r_range)
                  % if range limits are in place look for
                  % any landmarks that match criteria

                  % get range/bearing to all landmarks
                  z = s.h(s.robot.x');
                  jf = 1:numcols(s.map.map);

                  if ~isempty(s.r_range)
                      % find all within range
                      k = find(z(:,1) >= s.r_range(1) & z(:,1) <= s.r_range(2));
                      z = z(k,:);
                      jf = jf(k);
                  end

                  if isempty(k)
                      % no landmarks found
                      z = [];
                      jf = NaN;
                  elseif length(k) >= 1
                      % more than 1 in range, pick a random one
                      i = s.randstream.randi(length(k));
                      z = z(i,:);
                      jf = jf(i);
                  end

              else
                  % randomly choose the feature
                  %jf = s.selectFeature();
                  jf = 1;
                  % compute the range and bearing from robot to feature
                  z = s.h(s.robot.x);   
              end

              if s.verbose
                  fprintf('Sensor:: feature %d: %.1f %.1f\n', k, z);
              end
              if ~isempty(z) & s.animate
                  s.plot(jf);
              end
            end
        end


        function z = h(s, xv)
            %RangeSensor.h Landmark range and bearing
            %
            % Z = S.h(XV) is a sensor observation scalar range, from vehicle at 
            % pose XV (1x3) to the map feature K.
            %
            % Z = S.h(XV, XF) as above but compute range and bearing to a feature at coordinate XF.
            %
            % Notes::
            % - Noise with covariance W is added to each row of Z.
            % - Supports vectorized operation where XV (Nx3) and Z (Nx2).
            %
            % See also RangeSensor.Hx, RangeSensor.Hw, RangeSensor.Hxf.
          
            [polygon, intersection, z] = s.map.getRange(xv);
            if(z == Inf)
              z = 999999; % hack
            end
            
            % Model the lack of precision in the Fluke sensor
            if(~isempty(s.log))
              if(z < 0.245)
                z = 0.245;
              end
              if(z > 0.55)
                z = 999999;
              end
            end

            % add noise with covariance W
            z = z + s.randstream.randn(size(z)) * sqrt(s.W);
        end
        
        function xf = g(s, xv, z)
            %RangeSensor.g Compute landmark location
            %
            % P = S.g(XV, Z) is the world coordinate (1x2) of a feature given
            % the sensor observation Z (1x2) and vehicle state XV (3x1).
            %
            % See also RangeSensor.Gx, RangeSensor.Gz.

            range = z(1);

            xf = [xv(1)+range*cos(xv(3)) xv(2)+range*sin(xv(3))];
        end
        
        function weights = weight(s,x,z,jf,L)
          % score the particles

          % Straightforward code:
          %
          %  for p = 1:length(x)
          %    % what do we expect observation to be for this particle?
          %    % use the sensor model h(.)
          %    z_pred = sensor.h( x(p,:), jf);
          %    
          %    % how different is it
          %    innov(1) = z(1) - z_pred(1);
          %    
          %    % get likelihood (new importance). Assume Gaussian but any PDF works!
          %    % If predicted obs is very different from actual obs this score will be low
          %    %  ie. this particle is not very good at predicting the observation.
          %    % A lower score means it is less likely to be selected for the next generation...
          %    % The weight is never zero.
          %    weight(p) = exp(-0.5*innov'*inv(L)*innov) + 0.05;
          % end
          weights = zeros(length(x),1);
          
          for p = 1:length(x)
              z_pred = s.h( transpose(x(p,:)));
              q = z - z_pred;
              weights(p) = exp(-0.5*q'*inv(L)*q) + 0.05;
          end
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %              TODO: Implement this function!
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
        end

        function str = char(s)
            str = char@Sensor(s);
            str = char(str, ['W = ', mat2str(s.W, 3)]);

            str = char(str, sprintf('interval %d samples', s.interval) );
            if ~isempty(s.r_range)
                str = char(str, sprintf('range: %g to %g', s.r_range) );
            end

        end
        
    end % method
end % classdef
