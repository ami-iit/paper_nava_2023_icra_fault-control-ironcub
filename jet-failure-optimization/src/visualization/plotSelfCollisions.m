function plotSelfCollisions(centers, radiuses, figNum)

    % PLOTSELFCOLLISIONS plots the collisions as spheres wrapping the links.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    for k = 1:size(centers, 1)

        % plot the sphere
        figure(figNum)
        hold on

        % plot the center of the sphere
        plot3(centers(k,1), centers(k,2), centers(k,3), '.k', 'markersize', 20)

        % plot the sphere from radius
        [x,y,z] = sphere(10);
        x       = x*radiuses(k);
        y       = y*radiuses(k);
        z       = z*radiuses(k);
        surf(x+centers(k,1), y+centers(k,2), z+centers(k,3))
    end
end