function [out] = is_local_path_collision_free(q,q_rand)
    out = true;
    for t = 0.01:0.01:1
        q_init = interpolate(q,q_rand,t);
        if is_colliding(q_init)
            out=false;
            break
        end
    end
end