function [q_rand] = sample_nearby(q,q_rand)
    q_rand = reshape(q_rand,size(q));
    q_temp = 0.0;

    while(true)
        for i = 1:length(q_rand)
            q_temp = q(i)+(-1+2*rand(1,1));

            if( i== 0 || i == 1)
                if(q_temp > 8 || q_temp < -8)
                    q_temp = q(i)+(-1+2*rand(1,1));
                end
            end
            q_rand(i) = q_temp;
        end
        if is_local_path_collision_free(q,q_rand) == 1
            break;
        end
    end
end
    
    