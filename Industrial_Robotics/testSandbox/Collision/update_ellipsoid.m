R = ur3e_modified;
function ellipsoid(R,q)
    tr = zeros(4,4,R.model.n+1);
    tr(:,:,1) = R.model.base.T;
    
    L = R.model.links;
    for i = 1 : R.model.n
        tr(:,:,i+1) = (tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha));
    end
    
    for i = 1 : R.model.n
        if i == 1
            [X1, Y1, Z1] = ellipsoid(0, 0, 0.1, 0.06, 0.06, 0.125);
            mesh(X1,Y1,Z1);
        elseif i == 2
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X2, Y2, Z2] = ellipsoid(mid_point(1), mid_point(2)-0.125, mid_point(3), 0.2, 0.06, 0.06);
            mesh(X2,Y2,Z2);
        elseif i == 3
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X3, Y3, Z3] = ellipsoid(mid_point(1), mid_point(2)-0.03, mid_point(3), 0.125, 0.06, 0.06);
            mesh(X3,Y3,Z3);
        elseif i == 4
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X4, Y4, Z4] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.075, 0.06);
            mesh(X4,Y4,Z4);
        elseif i == 5
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X5, Y5, Z5] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.06, 0.1);
            mesh(X5,Y5,Z5);
        else
            mid_point = (tr(1:3, 4, i+1) + tr(1:3, 4, i)) / 2;
            [X6, Y6, Z6] = ellipsoid(mid_point(1), mid_point(2), mid_point(3), 0.06, 0.05, 0.06);
            mesh(X6,Y6,Z6);
        end
    end
end