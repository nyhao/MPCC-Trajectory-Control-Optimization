classdef GPS2MeterConverter
    %GPS2MeterConversion Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        origin_long;
        origin_lat; 
        origin_alt; 
        cos_origin_lat;
        R = 6371 * 1000;  % radius of earth in meter
    end
    
    methods
        function obj = GPS2MeterConverter(longitude, latitude, altitude)
            obj.cos_origin_lat = cos(deg2rad(latitude));
            obj.origin_long = obj.R * deg2rad(longitude) * obj.cos_origin_lat;
            obj.origin_lat = obj.R * deg2rad(latitude);
            obj.origin_alt = altitude;
        end
        
        function [x, y, z] = gps_to_meter(obj, longitude, latitude, altitude)
            x = obj.R * deg2rad(longitude) * obj.cos_origin_lat - obj.origin_long;
            y = obj.R * deg2rad(latitude) - obj.origin_lat;
            z = altitude - obj.origin_alt;
        end
        
        function [longitude, latitude, altitude, heading, tilt] = meter_to_gps(obj, x, y, z, yaw, camyaw, campitch)
            longitude = rad2deg((x + obj.origin_long) / (obj.R * obj.cos_origin_lat));
            latitude = rad2deg((y + obj.origin_lat) / obj.R);
            altitude = z + obj.origin_alt;
            heading = rad2deg(yaw + camyaw);
            tilt = rad2deg(campitch);
        end
    end
    
end

