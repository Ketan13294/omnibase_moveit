function [qt] = interpolate(qA,qB,t)
    qt = zeros(size(qA));
    for i = 1:length(qB)
        qB(i) = (1.0-t)*qA(i)+t*qB(i);
    end
end
