function bnch_print = Bench_Print(location_BPrint)

    bnch_print = PlaceObject('Bench.ply',location_BPrint);
    verts = [get(bnch_print,'Vertices'), ones(size(get(bnch_print,'Vertices'),1),1)];
    verts(:,1) = verts(:,1); %x
    verts(:,2) = verts(:,2); %y
    verts(:,3) = verts(:,3); %z
    set(bnch_print,'Vertices',verts(:,1:3))
    
    bnch_print_colors = [
        0.60, 0.80, 0.20; % Olive Drab
    ];

    num_faces = size(get(bnch_print, 'Faces'), 1);
    bnch_print_face_colors = bnch_print_colors(randi(size(bnch_print_colors, 1), [num_faces, 1]), :);
    
    patch('Vertices', get(bnch_print, 'Vertices'), 'Faces', get(bnch_print, 'Faces'), 'FaceVertexCData', bnch_print_face_colors, 'FaceColor', 'flat');
    colormap(bnch_print_colors);

end