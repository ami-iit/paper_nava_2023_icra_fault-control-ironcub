function s = similarityFcn(x1, x2, WeightCollision)

    s = exp(-0.5*transpose(x1-x2)*WeightCollision*(x1-x2));
end