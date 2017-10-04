classdef TrajHistory < handle
    properties
        data
    end
    
    methods
        function obj = TrajHistory(num_states, num_times, num_runs)
            obj.data = zeros(num_states, num_times, num_runs, 'int8');
        end
    end
end
    