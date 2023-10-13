function bnch_omran = Bench_Omran(location_BOmran)

    bnch_omran = PlaceObject('Bench.ply',location_BOmran);
    % verts = [get(bnch_omran,'Vertices'), ones(size(get(bnch_omran,'Vertices'),1),1)] * trotz(pi);
    % verts(:,1) = verts(:,1); %x
    % verts(:,2) = verts(:,2); %y
    % verts(:,3) = verts(:,3); %z
    % set(bnch_omran,'Vertices',verts(:,1:3))
    % 
    % bnch_omran_colors = [
    %     0.13, 0.55, 0.13; % Dark Green
    % ];
    % 
    % num_faces = size(get(bnch_omran, 'Faces'), 1);
    % bnch_omran_face_colors = bnch_omran_colors(randi(size(bnch_omran_colors, 1), [num_faces, 1]), :);
    % 
    % patch('Vertices', get(bnch_omran, 'Vertices'), 'Faces', get(bnch_omran, 'Faces'), 'FaceVertexCData', bnch_omran_face_colors, 'FaceColor', 'flat');
    % colormap(bnch_omran_colors);

end