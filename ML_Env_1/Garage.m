function grg = Garage(location_Garage)

    grg = PlaceObject('Garage.ply',location_Garage);
    verts = [get(grg,'Vertices'), ones(size(get(grg,'Vertices'),1),1)];
    verts(:,1) = verts(:,1); %x
    verts(:,2) = verts(:,2); %y
    verts(:,3) = verts(:,3); %z
    set(grg,'Vertices',verts(:,1:3))
    
    grg_colors = [
        0.36, 0.48, 0.53  % Navy Gray color
    ];

    num_faces = size(get(grg, 'Faces'), 1);
    grg_face_colors = grg_colors(randi(size(grg_colors, 1), [num_faces, 1]), :);
    
    patch('Vertices', get(grg, 'Vertices'), 'Faces', get(grg, 'Faces'), 'FaceVertexCData', grg_face_colors, 'FaceColor', 'flat');
    colormap(grg_colors);

end