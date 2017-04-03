function timingfunc = sumtanh(ps, ig, xr, ti, ntp)
    syms f pro
    f = ps(1) + ps(end);
    for k = 1:(ntp)
        f = f + (ps(k+1)-ps(k))*tanh(ig*pro-xr(5,ti(k))*ig);
    end
    f = 0.5*f;
    timingfunc = matlabFunction(f);
end

function objective = one_shot_cost(z, par)
    kmax = 320;
    gap = 10;

    pos = z(1:3);
    yaw = z(4);
    vel = z(5:7);
    yaw_rate = z(8);
    jerk = z(9:12);
    theta = z(13);
    theta_rate = z(14);
    theta_jerk = z(15);
    
    theta_ref = par(1:(kmax/gap)+1);
    poly_coefs = par((kmax/gap)+2:end);
end
            