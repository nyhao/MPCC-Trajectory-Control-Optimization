function [longitude, latitude, altitude, heading, tilt] = ... 
    coordinate_to_gps(start_lon, start_lat, start_alt, x, y, z, yaw, camyaw, campitch)

    R = 6371 * 1000;  % radius of earth in meter
    cos_origin_lat = cos(deg2rad(start_lat));
    origin_lon = R * deg2rad(start_lon) * cos_origin_lat;
    origin_lat = R * deg2rad(start_lat);
    origin_alt = start_alt;
    
    longitude = rad2deg((x + origin_lon) / (R * cos_origin_lat));
    latitude = rad2deg((y + origin_lat) / R);
    altitude = z + origin_alt;
    heading = rad2deg(yaw + camyaw);
    tilt = rad2deg(campitch);

end