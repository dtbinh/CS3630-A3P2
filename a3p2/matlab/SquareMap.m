%map Map of planar point features
%
% A Map object represents a square 2D environment with a square obstacles
%
% Methods::
%   plot      Plot the feature map
%   squares   nx4 array of x,y,w,h parameters for squares
%   display   Display map parameters in human readable form
%   char      Convert map parameters to human readable string
%
% Properties::
%   map         Matrix of map feature coordinates 2xN
%   dim         The dimensions of the map region x,y in [-dim,dim]
%   nsquares   The number of map features N
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also RangeBearingSensor, EKF.

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

classdef SquareMap < handle
% TODO:
% add a name property, show in char()

    properties
        map    % map features
        dim         % map dimension
        squares   % squares in map
        verbose
        polys
        norms
        d
    end

    methods

        % constructor
        function map = SquareMap(squares, varargin)
        %Map.Map Map of point feature landmarks
        %
        %
        % Options::
        % 'verbose'    Be verbose
            
            if(size(squares,2) ~= 4)
              error('Squares argument only takes nx4 arrays!')
            end
            opt = [];
            [opt,args] = tb_optparse(opt, varargin);
            map.verbose = opt.verbose;

            if ~isempty(args) && isnumeric(args{1})
                dim = args{1};
            else
                dim = 10;
            end
            map.dim = dim;
            map.squares = squares;

            map.verbose = false;
            map.parseWalls();
            
        end
        
        function parseWalls(map)    
          %Generates a list of polygons used for raytracing
          indexOffset = 0;
          Vs = [];
          Ps = [];
          for i=1:size(map.squares,1)
            % Compute vertices given x,y,w,h
            x1 = map.squares(i,1);
            y1 = map.squares(i,2);
            x2 = x1 + map.squares(i,3);
            y2 = y1 + map.squares(i,4);

            % Make sure to return to the origin
            X = [x1 x2 x2 x1 x1];
            Y = [y1 y1 y2 y2 y1];

            [V,P] = wall(X,Y);
            P = P+indexOffset;

            Vs = [Vs V];
            Ps = [Ps P];

            % Offset by 2*n where n is length(X)
            indexOffset = indexOffset + 10;
          end
                      
          [VT,PT] = triangulate(Vs,Ps);
          [N,d] = normals(VT,PT);
          T = polygonT(VT,PT);
          map.polys = T;
          map.norms = N;
          map.d = d;
          map.norms
        end
        
        function [polygon,intersection,t] = getRange(map, X)
          %Map.getRange Return range t along with intersection point on
          %polygon
          %X is 1x3 vector describing robot pose
          [polygon,intersection,t] = raytrace(map.norms,map.d,map.polys,....
            [X(1:2);0],[cos(X(3));sin(X(3));0]);
        end

        function f = feature(map, k)
            %Map.feature Return the specified map feature
            %
            % F = M.feature(K) is the coordinate (2x1) of the K'th feature.
            f = map.map(:,k);
        end

        function plot(map, varargin)
            %Map.plot Plot the map
            %
            % M.plot() plots the feature map in the current figure, as a square
            % region with dimensions given by the M.dim property.  Each feature
            % is marked by a black diamond.
            %
            % M.plot(LS) plots the feature map as above, but the arguments LS
            % are passed to plot and override the default marker style.
            %
            % Notes::
            % - The plot is left with HOLD ON.
            clf
            d = map.dim;
            axis([-d d -d d]);
            xlabel('x');
            ylabel('y');
            grid on
            for i=1:size(map.squares)
              rectangle('Position', map.squares(i,:))
              hold on
            end
            
        end

        function show(map, varargin)
        %map.SHOW Show the feature map
        %
        % Notes::
        % - Deprecated, use plot method.
            warning('show method is deprecated, use plot() instead');
            map.plot(varargin{:});
        end

        function verbosity(map, v)
            %map.verbosity Set verbosity
            %
            % M.verbosity(V) set verbosity to V, where 0 is silent and greater
            % values display more information.
            map.verbose = v;
        end
            
        function display(map)
            %map.display Display map parameters
            %
            % M.display() display map parameters in a compact
            % human readable form.
            %
            % Notes::
            % - this method is invoked implicitly at the command line when the result
            %   of an expression is a Map object and the command has no trailing
            %   semicolon.
            %
            % See also map.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(map) );
        end % display()

        function s = char(map)
        %map.char Convert vehicle parameters and state to a string
        %
        % s = M.char() is a string showing map parameters in 
        % a compact human readable format. 
            s = 'Map object';
            s = char(s, sprintf('  %d squares', size(map.squares,1)));
            s = char(s, sprintf('  dimension %.1f', map.dim));
        end

    end % method

end % classdef
