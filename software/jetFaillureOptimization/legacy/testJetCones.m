% test jet cones
clear
close all
clc

figure(1)

incrRadius = 0;
redOffset  = 0;

for k=1:10

    radius  = 0.15 + incrRadius;
    offset  = 0 + redOffset;
    [x,y,z] = sphere(10);

    if k > 0
        x       = x(6:end,:)*radius;
        y       = y(6:end,:)*radius;
        z       = z(6:end,:)*radius;
    else
        x       = x*radius;
        y       = y*radius;
        z       = z*radius;
    end
    surf(x, y, z+offset)
    axis equal
    hold on

    incrRadius = incrRadius + 0.05;
    redOffset  = redOffset - 0.2;

end
