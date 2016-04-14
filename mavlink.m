%% MAVLINK QGroundControl MATLAB interface
%
%   This class implements a very few Mavlink messages (originally intended
%   for use with QGroundControl software). For the moment, we are able to
%   only send messages. Future work shall deliver the possibility to
%   receive messages as well.
%
%   Additionally, it seems to the author that byte manipulation with FI
%   class is quite ineficient computacionally. Future work shall test other
%   data structures to assemble and disassmble UDP packets.
%
%   Constructor function takes the ground station IP address as input. We
%   consider the port as the traditional QGroundStation one: 14550, and the
%   local port as 14549. NOTICE: the IP is in string format, for example:
%   '192.168.0.1'
%

classdef mavlink < handle
    
    properties (GetAccess='private', SetAccess='private')
        %% udp socket descriptor for QGroundControl
        udp_socket = [];
        %% packet counter in the MAVLink protocol
        package_sequence = fi(0, false, 8, 0);
        %% ground station ip
        ip = '';
    end
    
    properties (Constant, GetAccess='private')
        % Constant for checksum computations
        X25_INIT_CRC_HEX = 'FFFF';
        % ID for MAVLink component and MAV identification
        MATLAB_ID = 1; % whatever number works here
        
        %% MAVLink definitions
        % Packet parameters
        PACKET_START_HEX = 'FE'; % start flag
        % Heartbeat parameters
        HEARTBEAT_LENGTH = 9; % 9 bytes of data
        HEARTBEAT_ID = 0;
        HEARTBEAT_CRC = 50;
        % "Set new GNSS Origin" message parameters
        GNSS_ORIGIN_LENGTH = 12;
        GNSS_ORIGIN_ID = 49;
        GNSS_ORIGIN_CRC = 39;
        % Local Position NED message parameters
        LOCAL_POSITION_NED_LENGTH = 28;
        LOCAL_POSITION_NED_ID = 32;
        LOCAL_POSITION_NED_CRC = 185;
        % Navigation system message parameters
        GLOBAL_POSITION_INT_LENGTH = 28;
        GLOBAL_POSITION_INT_ID = 33;
        GLOBAL_POSITION_INT_CRC = 104;
        % Attitude message parameters
        ATTITUDE_LENGTH = 28;
        ATTITUDE_ID = 30;
        ATTITUDE_CRC = 39;
        
     end
    
    methods
        
        function obj = mavlink(ip)
            %% MAVLINK: constructor takes the ground station IP as input.
            %
            % Example of usage: obj = mavlink('192.168.0.1');
            
            %% binds udp socket
            obj.ip = ip;
            obj.udp_socket = udp(obj.ip, 14550, 'LocalPort', 14549); % this is QGroundControl IP
            fopen(obj.udp_socket);
        end
        
        function delete(obj)
            %% DELETE: house-keeping tasks
            obj.closeConnection();
        end
        
        function closeConnection(obj)
            %% CLOSECONNECTION: closes udp socket
            %
            % This function takes no inputs and no outputs
            
            %% closes socket
            fclose(obj.udp_socket);
        end
        
        function sendHeartBeat(obj)
            %% SENDHEARTBEAT: sends QGroundControl heartbeat
            %
            % This function implements QGroundControl heartbeat. It is not
            % necessary for other messages to work though. QGroundControl
            % still receives and updates the station without a periodic
            % heartbeat.
            
            payload_data = fi(1:obj.HEARTBEAT_LENGTH,false,8,0);
            % uint32_t Custom Mode
            payload_data.data(1) = 0;
            payload_data.data(2) = 0;
            payload_data.data(3) = 0;
            payload_data.data(4) = 0;
            % uint8_t Type of MAV, defined in MAV_TYPE ENUM
            payload_data.data(5) = 1; 
            % uint8_t Autopilot type, defined in MAV_AUTOPILOT ENUM
            payload_data.data(6) = 3;
            % uint8_t System mode bitfield, defined in MAV_MODE_FLAG ENUM
            payload_data.data(7) = 81;
            % uint8_t System Status Flag, see MAV_STATE ENUM
            payload_data.data(8) = 4;
            % uint8_t MAVLink version, not writable by user (DO NOT CHANGE THIS!)
            payload_data.data(9) = 3;
            
            sendUDPPacket(obj,obj.HEARTBEAT_LENGTH, payload_data, obj.HEARTBEAT_ID, obj.HEARTBEAT_CRC);
            
        end
        
        function sendNavState(obj, time, lat, lon, alt, rel_alt, vx, vy, vz, heading)
            %% SENDNAVSTATE: send navigation variables via mavlink
            %
            % inputs: time - Scalar Real - current time (secs)
            %         lat - Scalar Real - WGS84 latitude (rad)
            %         lon - Scalar Real - WGS84 longitude (rad)
            %         alt - Scalar Real - altitude (not WGS64) (meters)
            %         rel_alt - Scalar Real - relative altitude (m)
            %         vx - Scalar Real - north velocity (m/s)
            %         vy - Scalar Real - east velocity (m/s)
            %         vz - Scalar Real - down velocity (m/s)
            %         heading - Scalar Real - i have no idea, QGroundControl ignores it.
            %
            % NOTICE: in QGroundControl, this message changes the position
            % in the map, but the heading information seems to do nothing,
            % velocity and LLA information additionally appears in the
            % unmanned systems wigdet.
            
            % time stamp (miliseconds since boot)
            time_stamp = fi([],false,32,0);
            time_stamp.data = floor(time*1000); % this needs to be in ms
            % latitude (*1e7, degrees)
            payload_lat = fi([],true,32,0);
            payload_lat.data = floor(lat*180/pi*1e7);
            % longitude (*1e7, degrees)
            payload_lon = fi([],true,32,0);
            payload_lon.data = floor(lon*180/pi*1e7);
            % altitude (in mm)
            payload_alt = fi([],true,32,0);
            payload_alt.data = floor(alt*1e3);
            % relative altitude (in mm)
            payload_rel_alt = fi([],true,32,0);
            payload_rel_alt.data = floor(rel_alt*1e3);
            % int16_t velocity in x (m/s*100)
            payload_vx = fi([],true,16,0);
            payload_vx.data = floor(vx*100);
            % int16_t velocity in y (m/s*100)
            payload_vy = fi([],true,16,0);
            payload_vy.data = floor(vy*100);
            % int16_t velocity in z (m/s*100)
            payload_vz = fi([],true,16,0);
            payload_vz.data = floor(vz*100);
            % uint16_t heading in degrees *100 (0 to 359.99)
            payload_head = fi([],false,16,0);
            payload_head.data = floor(heading*180/pi*100);
            
            payload_data = fi(1:obj.GLOBAL_POSITION_INT_LENGTH,false,8,0);
            
            % uint32_t
            init = 1; fin = 8; 
            payload_data.bin(init:fin) = time_stamp.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int32_t
            payload_data.bin(init:fin) = payload_lat.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lat.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lat.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lat.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int32_t
            payload_data.bin(init:fin) = payload_lon.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lon.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lon.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_lon.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int32_t
            payload_data.bin(init:fin) = payload_alt.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_alt.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_alt.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_alt.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int32_t
            payload_data.bin(init:fin) = payload_rel_alt.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_rel_alt.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_rel_alt.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_rel_alt.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int16_t
            payload_data.bin(init:fin) = payload_vx.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vx.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int16_t
            payload_data.bin(init:fin) = payload_vy.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vy.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int16_t
            payload_data.bin(init:fin) = payload_vz.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vz.bin(1:8);
            init = init + 11; fin = fin + 11;
            % int16_t
            payload_data.bin(init:fin) = payload_head.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_head.bin(1:8);
            init = init + 11; fin = fin + 11;
            
            sendUDPPacket(obj,obj.GLOBAL_POSITION_INT_LENGTH, payload_data, obj.GLOBAL_POSITION_INT_ID, obj.GLOBAL_POSITION_INT_CRC);
            
        end
        
        function sendAttitude(obj, time, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate)
            %% SENDATTITUDE: send attitude message in euler angles
            %
            % inputs: time - Scalar Real - current time (secs)
            %         roll - Scalar Real - roll (rad)
            %         pitch - Scalar Real - pitch (rad)
            %         yaw - Scalar Real - yaw (meters)
            %         roll_rate - Scalar Real - roll rate (rad/s)
            %         pitch_rate - Scalar Real - pitch rate (rad/s)
            %         yaw_rate - Scalar Real - yaw rate (rad/s)
            %
            % NOTICE: this message makes the artificial horizon in
            % QGroundControl move and also dictates heading in Map View
            % wigdet.
            
            payload_data = fi(1:obj.ATTITUDE_LENGTH,false,8,0);
            
            % time stamp computation
            time_stamp = fi([],false,32,0);
            time_stamp.data = floor(time*1000); % this needs to be in ms
            % roll
            payload_roll = fi([],true,32,0);
            payload_roll.data = typecast(single(roll), 'int32'); 
            % y
            payload_pitch = fi([],true,32,0);
            payload_pitch.data = typecast(single(pitch), 'int32'); 
            % z
            payload_yaw = fi([],true,32,0);
            payload_yaw.data = typecast(single(yaw), 'int32'); 
            % vx
            payload_roll_rate = fi([],true,32,0);
            payload_roll_rate.data = typecast(single(roll_rate), 'int32'); 
            % vy
            payload_pitch_rate = fi([],true,32,0);
            payload_pitch_rate.data = typecast(single(pitch_rate), 'int32'); 
            % vz
            payload_yaw_rate = fi([],true,32,0);
            payload_yaw_rate.data = typecast(single(yaw_rate), 'int32'); 
            
            % uint32_t Timestamp, in msecs
            init = 1; fin = 8; 
            payload_data.bin(init:fin) = time_stamp.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_roll.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_pitch.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_yaw.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_roll_rate.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll_rate.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll_rate.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_roll_rate.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_pitch_rate.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch_rate.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch_rate.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_pitch_rate.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_yaw_rate.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw_rate.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw_rate.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_yaw_rate.bin(1:8);
            init = init + 11; fin = fin + 11;
            
            sendUDPPacket(obj,obj.ATTITUDE_LENGTH, payload_data, obj.ATTITUDE_ID, obj.ATTITUDE_CRC);
            
        end
        
        function sendGNSSOrigin(obj, lat, lon, alt)
            %% SENDGNSSORIGIN: send GNSS origin message
            %
            % inputs: lat - Scalar Real - WGS84 latitude (rad)
            %         lon - Scalar Real - WGS84 longitude (rad)
            %         alt - Scalar Real - (not WGS84) altitude (m)
            %
            % NOTICE: i couldn't find a use for this in QGroundControl. I
            % implemented it in the hope of changing the position of the
            % UAV in Map View with NED local coordinates (XYZ), but it
            % didn't work. Weird, need to check this up later. In the
            % meantime, please use sendNavState() for printing position in
            % Map View. I am still searching for a solution for ploting
            % directly XYZ position in a local NED.
            
            
            lat_mavlink = fi([],true,32,0);
            lon_mavlink = fi([],true,32,0);
            alt_mavlink = fi([],true,32,0);
            
            lat_mavlink.data = floor(180/pi*lat*1e7);
            lon_mavlink.data = floor(180/pi*lon*1e7);
            alt_mavlink.data = floor(alt*1000);
            
            payload_data = fi(1:obj.GNSS_ORIGIN_LENGTH,false,8,0);
            % uint32_t Latitude (WGS84), in degrees * 1E7
            init = 1; fin = 8; 
            payload_data.bin(init:fin) = lat_mavlink.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lat_mavlink.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lat_mavlink.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lat_mavlink.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Longitude (WGS84), in degrees * 1E7
            payload_data.bin(init:fin) = lon_mavlink.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lon_mavlink.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lon_mavlink.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = lon_mavlink.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Altitude (AMSL), in meters * 1000 (positive for up)
            payload_data.bin(init:fin) = alt_mavlink.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = alt_mavlink.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = alt_mavlink.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = alt_mavlink.bin(1:8);
            
            sendUDPPacket(obj,obj.GNSS_ORIGIN_LENGTH, payload_data, obj.GNSS_ORIGIN_ID, obj.GNSS_ORIGIN_CRC);
            
        end
        
        function sendLocalPosition(obj, time, x, y, z, vx, vy, vz)
            %% SENDLOCALPOSITION: send local position message
            %
            % inputs: x - Scalar Real - north position (m)
            %         y - Scalar Real - east position (m)
            %         z - Scalar Real - down position (m)
            %         vx - Scalar Real - north velocity (m/s)
            %         vy - Scalar Real - east velocity (m/s)
            %         vz - Scalar Real - down velocity (m/s)
            %
            % NOTICE: i couldn't find a use for this in QGroundControl. I
            % implemented it in the hope of changing the position of the
            % UAV in Map View with NED local coordinates (XYZ), but it
            % didn't work. Weird, need to check this up later. In the
            % meantime, please use sendNavState() for printing position in
            % Map View. I am still searching for a solution for ploting
            % directly XYZ position in a local NED. Nevertheless, the
            % positions and velocity XYZ that are sent with this are shown 
            % in the unmanned systems wigdet.
            
            payload_data = fi(1:obj.LOCAL_POSITION_NED_LENGTH,false,8,0);
            
            % time stamp computation
            time_stamp = fi([],false,32,0);
            time_stamp.data = floor(time*1000); % this needs to be in ms
            % x
            payload_x = fi([],true,32,0);
            payload_x.data = typecast(single(x), 'int32'); 
            % y
            payload_y = fi([],true,32,0);
            payload_y.data = typecast(single(y), 'int32'); 
            % z
            payload_z = fi([],true,32,0);
            payload_z.data = typecast(single(z), 'int32'); 
            % vx
            payload_vx = fi([],true,32,0);
            payload_vx.data = typecast(single(vx), 'int32'); 
            % vy
            payload_vy = fi([],true,32,0);
            payload_vy.data = typecast(single(vy), 'int32'); 
            % vz
            payload_vz = fi([],true,32,0);
            payload_vz.data = typecast(single(vz), 'int32'); 
            
            % uint32_t Timestamp, in msecs
            init = 1; fin = 8; 
            payload_data.bin(init:fin) = time_stamp.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = time_stamp.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_x.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_x.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_x.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_x.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_y.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_y.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_y.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_y.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_z.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_z.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_z.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_z.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_vx.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vx.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vx.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vx.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_vy.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vy.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vy.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vy.bin(1:8);
            init = init + 11; fin = fin + 11;
            % uint32_t Timestamp, in msecs
            payload_data.bin(init:fin) = payload_vz.bin(25:32);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vz.bin(17:24);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vz.bin(9:16);
            init = init + 11; fin = fin + 11;
            payload_data.bin(init:fin) = payload_vz.bin(1:8);
            init = init + 11; fin = fin + 11;
            
            sendUDPPacket(obj,obj.LOCAL_POSITION_NED_LENGTH, payload_data, obj.LOCAL_POSITION_NED_ID, obj.LOCAL_POSITION_NED_CRC);
            
        end
        
    end
    
    methods (Access=private)
        
        function sendUDPPacket(obj, lenght, payload_data, msgID, extra_crc)
            %% SENDUDPPACKET: sends a generic mavlink UDP packet
            %
            % inputs: length - Scalar Real - mavlink length of packet in bytes
            %         payload_data - FI object - payload data
            %         msgID - Scalar Real - Message ID
            %         extra_crc - Scalar Real - Checksum extra CRC (depends on message ID)
            
            packet_start = fi([],false,8,0);
            packet_start.hex = obj.PACKET_START_HEX;
            
            payload_length = fi([],false,8,0);
            payload_length.data = lenght;
            
            obj.package_sequence.data = obj.package_sequence.data + 1;
            
            system_ID = fi([],false,8,0);
            system_ID.data = obj.MATLAB_ID;
            
            component_ID = fi([],false,8,0);
            component_ID.data = obj.MATLAB_ID;
            
            message_ID = fi([],false,8,0);
            message_ID.data = msgID;
            
            %% checksum computation
            % initial checksum
            checksum = fi([],false,16,0);
            checksum.hex = obj.X25_INIT_CRC_HEX;
            
            % checksum accumulations
            
            checksum = obj.crcAccumulate( payload_length, checksum );
            checksum = obj.crcAccumulate( obj.package_sequence, checksum );
            checksum = obj.crcAccumulate( system_ID, checksum );
            checksum = obj.crcAccumulate( component_ID, checksum );
            checksum = obj.crcAccumulate( message_ID, checksum );
            
            for i=1:lenght
                payload_entry = payload_data(i);
                checksum = obj.crcAccumulate( payload_entry, checksum );
            end
            
            extra_low = fi(extra_crc,false,8,0);
            
            checksum = obj.crcAccumulate( extra_low, checksum );
            
            %% constructs message to send through UDP
            msg = [];
            msg = [msg packet_start.data];
            msg = [msg, payload_length.data];
            msg = [msg, obj.package_sequence.data];
            msg = [msg, system_ID.data];
            msg = [msg, component_ID.data];
            msg = [msg, message_ID.data];
            for i = 1:payload_length.data
                msg = [msg, payload_data.data(i)];
            end
            check_high = fi([],false,8,0);
            check_low = fi([],false,8,0);
            check_high.bin = checksum.bin(1:8);
            check_low.bin = checksum.bin(9:16);
            msg = [msg, check_low.data];
            msg = [msg, check_high.data];
            
            fwrite(obj.udp_socket,msg);
            
        end
        
        function updatedCRC = crcAccumulate( ~, byte, checksum )
            %% CRCACCUMULATE Computes checksum for mavlink messages incrementally
            %
            % inputs: byte - 8 bit FI object - byte to be inserted in the checksum
            %         checksum - 16 bit FI object - previous checksum
            % outputs: updatedCRC - 16 bit FI object - new checksum
            
            hexValueFF = fi([],false,16,0);
            hexValueFF.hex = 'FF';
            
            data_16bits = fi([],false,16,0);
            data_16bits.data = 0;
            data_16bits.bin(9:16) = byte.bin;
            
            tmp = bitxor(data_16bits,bitand(checksum,hexValueFF));
            tmp = bitxor(tmp,bitand(bitsll(tmp,4),hexValueFF));
            
            P1 = bitxor(bitsrl(checksum,8),bitsll(tmp,8));
            P2 = bitxor(bitsll(tmp,3),bitsrl(tmp,4));
            updatedCRC = bitxor(P1,P2);
            
            
        end
        
    end

end


    
    
    