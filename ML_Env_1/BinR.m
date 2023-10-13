function bin = BinR(location_Bin)

    bin = PlaceObject('BinR.ply',location_Bin);
    verts = [get(bin,'Vertices'), ones(size(get(bin,'Vertices'),1),1)];
    verts(:,1) = verts(:,1); %x
    verts(:,2) = verts(:,2); %y
    verts(:,3) = verts(:,3); %z
    set(bin,'Vertices',verts(:,1:3))
    
    bin_colors = [
        1, 0.1, 0.2  % sss
    ];

    num_faces = size(get(bin, 'Faces'), 1);
    bin_face_colors = bin_colors(randi(size(bin_colors, 1), [num_faces, 1]), :);
    
    patch('Vertices', get(bin, 'Vertices'), 'Faces', get(bin, 'Faces'), 'FaceVertexCData', bin_face_colors, 'FaceColor', 'flat');
    colormap(bin_colors);

end