function StopB(input)
    stp = PlaceObject('emergencyStopButton.ply', input);
    verts = [get(stp,'Vertices'), ones(size(get(stp,'Vertices'),1),1)] * trotz(0/2) *trotx(pi/2);
    verts(:,1) = verts(:,1); %x
    verts(:,2) = verts(:,2); %y
    verts(:,3) = verts(:,3); %z
    set(stp,'Vertices',verts(:,1:3))

    new_color = [139, 69, 19] / 255; 

    % Create a patch object with the new color
    patch('Vertices', get(stp, 'Vertices'), 'Faces', get(stp, 'Faces'), 'FaceColor', new_color);
    
    % Define colors for camouflage pattern (e.g., shades of green and brown)
        camouflage_colors = [
        0.36, 0.25, 0.20; % Dark Brown
        0.55, 0.27, 0.07; % Saddle Brown
        0.54, 0.27, 0.08; % Sienna
    ];
    % Randomly apply the camouflage pattern colors to 'stp' faces
    num_faces = size(get(stp, 'Faces'), 1);
    camouflage_face_colors = camouflage_colors(randi(size(camouflage_colors, 1), [num_faces, 1]), :);
    
    % Apply the camouflage face colors to 'stp'
    patch('Vertices', get(stp, 'Vertices'), 'Faces', get(stp, 'Faces'), 'FaceVertexCData', camouflage_face_colors, 'FaceColor', 'flat');
    colormap(camouflage_colors);

end