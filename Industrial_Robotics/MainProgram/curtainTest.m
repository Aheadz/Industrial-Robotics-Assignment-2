function curtainTest
    %% Light Curtain
    % Define the range for x and z coordinates
    x = -2.7:0.1:2.7; % Change the range according to your requirement
    z = 0.324:0.1:1.35; % Change the range according to your requirement
    
    % Create a grid of coordinates for the plane
    [X, Z] = meshgrid(x, z);
    
    % Calculate the size of the grid
    [m, n] = size(X);
    
    % Adjust the y-coordinate of the light curtain
    y_offset = -1.0; % Change this value to adjust the y-coordinate
    Y = ones(size(X)) * y_offset;
    
    % Initialize the curtain_points array to store the points
    curtain_points = zeros(m * n, 3); % Initialize array to hold x, y, z coordinates
    
    % Plotting the 3D plane of green dots and storing the points
    index = 1;
    for i = 1:m
        for j = 1:n            
            % Storing the points in the curtain_points array
            curtain_points(index, 1) = X(i, j); % x-coordinate
            curtain_points(index, 2) = Y(i, j); % y-coordinate
            curtain_points(index, 3) = Z(i, j); % z-coordinate
            index = index + 1;
        end
    end
    %% Animate the movement and light curtain danger detect
        manx = linspace(0,0,50)';
        many = linspace(-2,0,50)';
        manTraj = ([manx,many,manx]);
    for step = 1:50
        man = PlaceObject('WpersonMaleConstruction.ply',manTraj(step,:));
        man_Points = get(man, 'Vertices');
        
        % Check if the man_Points are within 0.1 distance of curtain_Points
        if any(min(pdist2(man_Points, curtain_points)) < 0.1)
            disp('Man is in the danger!');
            disp('EStop has been triggered!');
            break; % Break out of the loop if the condition is met
        end 
        pause(0.1); % Pause to show the update
        delete(man)
    end
end