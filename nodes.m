%% HEADER
% robot.m by William Dakare

% Contains robot description class, and methods to define an isoperimetric
% truss robot. 

classdef nodes
    properties
         coordinates = [0 sqrt(12) 4;
                    0 2 0]';
         fixed =  [true, false, false];
        
    end


    methods  

 %% HELPER METHODS
     function [perimeter] = compute_perimeter(obj, coordinates)
            % Input type check. Must have the coordinate property
            prop = "coordinates"; 
            if ~isprop(obj, "coordinates")
                error("Object doesn' have attribute %s", prop);
            end
    
            perimeter = 0;
        
            for k = 1:length(coordinates)
            % Compute the perimeter using the coordinates. 
            % Edge cases:
            % - Don't take the norm at the end, take a closure distance between the
            % end and the first coordinate, for closure on the shape
                if k ~= length(coordinates) 
                    % Sequential edges
                    closure_edge = obj.compute_edge([coordinates(k, 1), coordinates(k, 2)], [coordinates(k + 1, 1), coordinates(k + 1, 2)]); % Compute the edge length between the first and last coordinate for closure on the shape
                    perimeter = perimeter + closure_edge;      
                else    
                    % Closing Edge
                    edge = obj.compute_edge([coordinates(1, 1), coordinates(1, 2)], [coordinates(k, 1), coordinates(k, 2)]); % Compute edge length between each coordinate
                    perimeter = perimeter + edge;
                end    
            end
        
     end
        
        
     function [edge_length] = compute_edge(~, d1, d2)
            edge_length = norm(d1 - d2);
     end
        
        
     %% Defining Isoperimeteric Constraint & Formatting Node coordinates
     function perimeter = perimeter_val(obj)
        node_coords = obj.format_coords;
        perimeter = obj.compute_perimeter(node_coords);
     end
    
     function node_coords = format_coords(obj)
        node_coords = obj.coordinates; % returns n x 2
     end
        
    end
    
end
