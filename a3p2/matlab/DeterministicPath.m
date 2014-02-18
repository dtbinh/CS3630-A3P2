%DeterministicPath Vehicle driver class

classdef DeterministicPath < handle
    properties
        veh
        log
        nrLines
        counter
    end
    
    methods
        
        function driver = DeterministicPath(filename)
            % Constructor
            driver.log = dlmread(filename,' ');
            driver.nrLines=size(driver.log,1)
            fprintf(1,'read %d lines from %s\n',driver.nrLines,filename);
            driver.counter=1;
        end
        
        function init(driver)
        end
        
        function u = demand(driver)
            % Return controls
            driver.counter = driver.counter + 1;
            if driver.counter>driver.nrLines
                u=[0;0];
            else
                % Motor values given to the Fluke [-200,200]
                % Left and right with respect to the Fluke
                u = [
                    driver.log(driver.counter,2)
                    driver.log(driver.counter,3)
                    ];
            end
        end
        
    end % methods
end % classdef
