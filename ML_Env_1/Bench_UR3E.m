function bnch_UR3e = Bench_UR3E(location_BUR3e)

    bnch_UR3e = PlaceObject('WBench.ply',location_BUR3e);
    verts = [get(bnch_UR3e,'Vertices'), ones(size(get(bnch_UR3e,'Vertices'),1),1)] * trotz(pi);
    verts(:,1) = verts(:,1); %x
    verts(:,2) = verts(:,2); %y
    verts(:,3) = verts(:,3); %z
    set(bnch_UR3e,'Vertices',verts(:,1:3))
    
    bnch_UR3e_colors = [
        0.33, 0.42, 0.18; % Dark Olive Green
    ];

    num_faces = size(get(bnch_UR3e, 'Faces'), 1);
    bnch_UR3e_face_colors = bnch_UR3e_colors(randi(size(bnch_UR3e_colors, 1), [num_faces, 1]), :);
    
    patch('Vertices', get(bnch_UR3e, 'Vertices'), 'Faces', get(bnch_UR3e, 'Faces'), 'FaceVertexCData', bnch_UR3e_face_colors, 'FaceColor', 'flat');
    colormap(bnch_UR3e_colors);

end