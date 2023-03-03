function isTouching = computeSelfCollisionDetector(KinDynModel, Config)

    % COMPUTESELFCOLLISIONDETECTOR check if two links are self colliding.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    isTouching = 0;

    for i = 1:length(Config.boundingBox.framesList)

        for j = 1:length(Config.boundingBox.framesList)

            % it suffices to check the lower triangular part of the matrix
            if j < i

                w_H_i       = iDynTreeWrappers.getWorldTransform(KinDynModel, Config.boundingBox.framesList{i});
                w_H_j       = iDynTreeWrappers.getWorldTransform(KinDynModel, Config.boundingBox.framesList{j});
                vertices_i  = Config.boundingBox.vertices(:,:,i);
                vertices_j  = Config.boundingBox.vertices(:,:,j);

                mesh_i      = collisionMesh(vertices_i);
                mesh_j      = collisionMesh(vertices_j);

                mesh_i.Pose = w_H_i;
                mesh_j.Pose = w_H_j;

                isTouching  = isTouching + checkCollision(mesh_i, mesh_j);
            end
        end
    end
end