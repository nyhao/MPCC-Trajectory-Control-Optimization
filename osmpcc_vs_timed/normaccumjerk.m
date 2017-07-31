function [timed_naj_pos, timed_naj_cam, mpcc_naj_pos, mpcc_naj_cam] = ...
    normaccumjerk(timed_trajectory, mpcc_trajectory, timed_size, mpcc_size)
    
    timed_accum_jerk_x = sum(abs(timed_trajectory(19,1:timed_size)))/timed_size;
    timed_accum_jerk_y = sum(abs(timed_trajectory(20,1:timed_size)))/timed_size;
    timed_accum_jerk_z = sum(abs(timed_trajectory(21,1:timed_size)))/timed_size;
    
    timed_naj_pos = norm([timed_accum_jerk_x, timed_accum_jerk_y, timed_accum_jerk_z]);

    timed_accum_jerk_yaw = sum(abs(timed_trajectory(33,1:timed_size)+timed_trajectory(22,1:timed_size)))/timed_size;
    timed_accum_jerk_pitch = sum(abs(timed_trajectory(34,1:timed_size)))/timed_size;
        
    timed_naj_cam = norm([timed_accum_jerk_yaw, timed_accum_jerk_pitch]);
    
    mpcc_accum_jerk_x = sum(abs(mpcc_trajectory(13,1:mpcc_size)))/mpcc_size;
    mpcc_accum_jerk_y = sum(abs(mpcc_trajectory(14,1:mpcc_size)))/mpcc_size;
    mpcc_accum_jerk_z = sum(abs(mpcc_trajectory(15,1:mpcc_size)))/mpcc_size;
    
    mpcc_naj_pos = norm([mpcc_accum_jerk_x, mpcc_accum_jerk_y, mpcc_accum_jerk_z]);
    
    mpcc_accum_jerk_yaw = sum(abs(mpcc_trajectory(25,1:mpcc_size)+mpcc_trajectory(16,1:mpcc_size)))/mpcc_size;
    mpcc_accum_jerk_pitch = sum(abs(mpcc_trajectory(26,1:mpcc_size)))/mpcc_size;
    
    mpcc_naj_cam = norm([mpcc_accum_jerk_yaw, mpcc_accum_jerk_pitch]);