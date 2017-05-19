function generate_kml_file(filename, longitude, latitude, altitude, heading, tilt)

    fileID = fopen(filename,'w');
    
    fprintf(fileID,'<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fileID,'<kml xmlns="http://www.opengis.net/kml/2.2"\n');
    fprintf(fileID,' xmlns:gx="http://www.google.com/kml/ext/2.2">\n');
    fprintf(fileID,'\n');
    fprintf(fileID,'<Document>\n');
    fprintf(fileID,'<gx:Tour>\n');
    fprintf(fileID,'<name>Tour</name>\n');
    fprintf(fileID,'<gx:Playlist>\n');
    
    for i = 1:numel(longitude)
        fprintf(fileID,'<gx:FlyTo>\n');
        fprintf(fileID,'<gx:duration>0.1</gx:duration>\n');
        fprintf(fileID,'<gx:flyToMode>smooth</gx:flyToMode>\n');
        fprintf(fileID,'<Camera>\n');
        fprintf(fileID,'<longitude>%f</longitude>\n',longitude(i));
        fprintf(fileID,'<latitude>%f</latitude>\n',latitude(i));
        fprintf(fileID,'<altitude>%f</altitude>\n',altitude(i));
        fprintf(fileID,'<heading>%f</heading>\n',heading(i));
        fprintf(fileID,'<tilt>%f</tilt>\n',tilt(i));
        fprintf(fileID,'<roll>0</roll>\n');
        fprintf(fileID,'<gx:horizFov>120</gx:horizFov>\n');
        fprintf(fileID,'<altitudeMode>absolute</altitudeMode>\n');
        fprintf(fileID,'</Camera>\n');
        fprintf(fileID,'</gx:FlyTo>\n');
        fprintf(fileID,'<gx:Wait>\n');
        fprintf(fileID,'<gx:duration>0.1</gx:duration>\n');
        fprintf(fileID,'</gx:Wait>\n');
    end
    
    fprintf(fileID,'</gx:Playlist>\n');
    fprintf(fileID,'</gx:Tour>\n');
    fprintf(fileID,'</Document>\n');
    fprintf(fileID,'</kml>\n');
    
    fclose(fileID);
    
end