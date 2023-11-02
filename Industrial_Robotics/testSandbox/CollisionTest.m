

%%

% hold on
% R = UR3e;
% 
% q = [0,0,0,0,0,0,0];
% 
% tr = zeros(4,4,R.model.n+1);
% tr(:,:,1) = R.model.base.T;
% tr(:,:,2) = trotz(pi) * transl(0,0,q(1)) * transl(0,0,0) * trotx(pi/2);
% 
% L = R.model.links;
% for i = 2 : R.model.n
%     tr(:,:,i+1) = (tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha));
% end
% 
% 
% % matZ = axang2tform([0 0 1 pi/2]);
% % box = collisionBox(0.5,1,0.1);
% % box.Pose = matZ;
% % show (box)
% 
% 
% 
% for i = 1 : R.model.n
%     [X, Y, Z] = ellipsoid(tr(1:1,4:4,i),tr(2:2,4:4,i),tr(3:3,4:4,i),0.05,0.05,0.1);
%     mesh(X, Y, Z);
% end
%%   
tbl = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0,0]);
vertex= get(tbl, 'Vertices');
faces = get(tbl, 'Faces');
faceNormals = get(tbl, 'Faces');
plotOptions.plotFaces = true;
R = ur3e_modified;
hold on
q = [0,0,0,0,0,0];

%update_ellipsoid(R,q);


   

%%
% for i = 1 : size(tr,3)-1    
%     for faceIndex = 1:size(faces,1)
%         vertOnPlane = vertex(faces(faceIndex,1)',:);
%         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%             display('Intersection');
%         end
%     end    
% end

%%
steps = 50;
Initial = R.model.fkine(R.model.getpos);
Final = Initial.T * transl(0,0,0.6);

qI = R.model.ikcon(Initial);
qF = R.model.ikcon(Final);

qTraj = jtraj(qI,qF,steps);
disp(qTraj)

for k = 1:steps
    % result(i) = IsCollision(R.model,qTraj(i,:),faces,vertex,faceNormals,false);
    R.model.animate(qTraj(k,:));
    drawnow();
end
update_ellipsoid(R,qTraj(steps,:));

%%  
%checkCollision(R,R.model.links.qlim,'SkippedSelfCollisions','parent')
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a R model (R), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(R,qTraj,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

    for qIndex = 1:size(qTraj,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(qTraj(qIndex,:), R);
    
        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    display('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end    
        end
    end
end


function update_ellipsoid(R,q)
    tr = zeros(4,4,R.model.n+1);
    tr(:,:,1) = R.model.base.T;
    
    L = R.model.links;
    for i = 1 : R.model.n
        tr(:,:,i+1) = (tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha));
    end
    
    for i = 1 : R.model.n
        if i == 1
            [X, Y, Z] = ellipsoid(0, 0, 0.1, 0.06, 0.06, 0.125);
        elseif i == 2
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X, Y, Z] = ellipsoid(mid_point(1), mid_point(2)-0.125, mid_point(3), 0.2, 0.06, 0.06);
        elseif i == 3
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X, Y, Z] = ellipsoid(mid_point(1), mid_point(2)-0.03, mid_point(3), 0.125, 0.06, 0.06);
        elseif i == 4
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X, Y, Z] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.075, 0.06);
        elseif i == 5
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X, Y, Z] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.06, 0.1);
        else
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X, Y, Z] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.05, 0.06);
        end
        mesh(X, Y, Z);
    end
end