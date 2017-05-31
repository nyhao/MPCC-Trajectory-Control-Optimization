function [longitude, latitude, altitude, heading, tilt] = ... 
    coordinate_to_gps(origin_lon, origin_lat, origin_alt, x, y, z, yaw, camyaw, campitch)

    coordinate_origin = zeros(4,1);
    cos_origin_lat = cos(origin_lat);
    R = 6371 * 1000;  % radius of earth in meter
    
    longitude = rad2deg((x + coordinate_origin(1)) / (R * cos_origin_lat)) + origin_lon;
    latitude = rad2deg((y + coordinate_origin(2)) / R) + origin_lat;
    altitude = z + coordinate_origin(3) + origin_alt;
    heading = rad2deg(yaw + camyaw);
    tilt = rad2deg(campitch);

end