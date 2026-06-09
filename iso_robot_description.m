%% HEADER
function [nodes] = iso_robot_definition
    nodes = struct('id', {1, 2, 3}, ...
               'coordinates', {[0,0], [2, sqrt(12)], [4,0]}, ...
               'fixed', {true, false, false});
    
end


function [perimeter] = compute_perimeter(nodes, coordinates)
    % Input type check. Must have the coordinate attribute
    if ~isfield(nodes, "coordinates")
        error("Wrong input type. Expected: Field Attribute")
    end
   
    perimeter = 0;

    for k = 1:length(coordinates)
    % Compute the perimeter using the coordinates. 
    % Edge cases:
    % - Don't take the norm at the end, take a closure distance between the
    % end and the first coordinate, for closure on the shape
        if k ~= length(coordinates) 
            closure_edge = compute_edge([coordinates(k, 1), coordinates(k, 2)], [coordinates(k + 1, 1), coordinates(k + 1, 2)]); % Compute the edge length between the first and last coordinate for closure on the shape
            perimeter = perimeter + closure_edge;      
        else    
            edge = compute_edge([coordinates(1, 1), coordinates(1, 2)], [coordinates(k, 1), coordinates(k, 2)]); % Compute edge length between each coordinate
            perimeter = perimeter + edge;
        end    
    end

end


function [edge_length] = compute_edge(d1, d2)
    edge_length = norm(d1 - d2);
end


%% Defining Isoperimeteric Constraint
function perimeter = perimeter_val(nodes)
% node_coords properly formats the input for the isoperimeter function
    % Variable definition
    coords_list = [nodes.coordinates];
    n_nodes = length(coords_list);

    if mod(n_nodes, 2) == 0
        % Pre-assign storage for nested list that preserves the [x, y] coordinate form for n/2 elements
        node_coords = repmat(zeros(1, 2), (n_nodes/2), 1);
    
        for k = 1:n_nodes/2
            node_coords(k, 1) = coords_list(2*k - 1);
            node_coords(k, 2) = coords_list(2*k);
        end
    else
        error("Wrong row vector length!")
    end
    perimeter = compute_perimeter(nodes, node_coords);
end


nodes = iso_robot_definition;
perimeter = perimeter_val(nodes)