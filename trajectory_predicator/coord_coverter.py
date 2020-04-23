
import numpy as np
from numpy import arcsin, sqrt, sin, cos, deg2rad
from data_parser import DataParser

#世界坐标系，以指定的 frame 时 ego-car 的位置为圆心，以 east 方向为 0 度角， north 方向为 90 度角
class CoordCoverter:
    def __init__(self, parse_dict, oxts_file):
        self.parse_dict = parse_dict
        self.oxts_file = oxts_file
        self.parseOxtsData()
        self.converted_coord_dict = dict()
        
    def setCoordOrigin(self,frame_id):
        self.origin_lat_lon_yaw = self.oxts_dict[frame_id]
        self.origin_pose = [0, 0, 0]
        
        
    def convertCoord(self, convert_frame_id_list):
        
        r = 6378137 #meters, r of earth-surface
        
        ''' 
        to calculate distance based on lat and lon
        
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        '''
        
        for convert_frame_id in convert_frame_id_list:
            current_oxts_coord = self.oxts_dict[convert_frame_id]
            
            
            moved_lat = deg2rad(current_oxts_coord[0]) - deg2rad(self.origin_lat_lon_yaw[0])
            moved_lon = deg2rad(current_oxts_coord[1]) - deg2rad(self.origin_lat_lon_yaw[1])
            
            
            moved_yaw_rad = current_oxts_coord[2] - self.origin_lat_lon_yaw[2]
            
            moved_meters_x =  r * 2 * arcsin(sqrt( cos( deg2rad(self.origin_lat_lon_yaw[0])) * deg2rad(cos(current_oxts_coord[0])) * sin(moved_lon/2)**2 )) 
            moved_meters_y =  r * 2 * arcsin(sqrt( sin(moved_lat/2)**2 ))
            
            if (moved_lon < 0):
                moved_meters_x = - moved_meters_x
            if (moved_lat < 0):
                moved_meters_y = - moved_meters_y
                
            new_origin_pose = [moved_meters_x, moved_meters_y, moved_yaw_rad] #新的原点坐标
            
            print("==================")
            print(str(convert_frame_id) + "th frame moved pose")
            print(new_origin_pose)
            print("==================")
            #接下来就是对 一个 frame 中所有的 detected car 
            for track_id in set(self.parse_dict[convert_frame_id].keys()):
                #track_id = object_info[0]
                object_type = self.parse_dict[convert_frame_id][track_id][0]
                shape = self.parse_dict[convert_frame_id][track_id][1]
                
                x = self.parse_dict[convert_frame_id][track_id][2][0]
                y = self.parse_dict[convert_frame_id][track_id][2][1]
                rotation = self.parse_dict[convert_frame_id][track_id][2][2]
                
                pose_after_converting = self.transform([x, y, rotation], new_origin_pose)
                
                if convert_frame_id not in set(self.converted_coord_dict.keys()):
                    self.converted_coord_dict[convert_frame_id] = {track_id:[object_type,shape,pose_after_converting]}
                else:
                    self.converted_coord_dict[convert_frame_id][track_id] = [object_type,shape,pose_after_converting]

                        
    def transform(self, pose_before_converting, new_origin_pose):
        moved_meters_x, moved_meters_y, moved_yaw_rad = new_origin_pose
        x_before, y_before, yaw_before = pose_before_converting
        
        r_mat = np.mat([ [np.cos(moved_yaw_rad), -np.sin(moved_yaw_rad), 0], 
                       [np.sin(moved_yaw_rad), np.cos(moved_yaw_rad),  0], 
                       [0,                            0,               1] ])

        t_mat = np.mat([ [1,    0,   moved_meters_x],
                         [0,    1,   moved_meters_y],
                         [0,    0,         1       ] ])
            
        new_coord_x_y_1 = t_mat * r_mat * np.mat([x_before, y_before, 1]).T 
        new_yaw = yaw_before + moved_yaw_rad
        
        pose_after_converting = [new_coord_x_y_1[0,0], new_coord_x_y_1[1,0], new_yaw]
        return pose_after_converting
        
    
    '''
      - lat:     latitude of the oxts-unit (deg)    纬度
      - lon:     longitude of the oxts-unit (deg)   经度
      - alt:     altitude of the oxts-unit (m)
      - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
      - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
      - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
      - vn:      velocity towards north (m/s)
      - ve:      velocity towards east (m/s)
      - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
      - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
      - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
      - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
      - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
      - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
      - af:      forward acceleration (m/s^2)
      - al:      leftward acceleration (m/s^2)
      - au:      upward acceleration (m/s^2)
      - wx:      angular rate around x (rad/s)
      - wy:      angular rate around y (rad/s)
      - wz:      angular rate around z (rad/s)
      - wf:      angular rate around forward axis (rad/s)
      - wl:      angular rate around leftward axis (rad/s)
      - wu:      angular rate around upward axis (rad/s)
      - posacc:  velocity accuracy (north/east in m)
      - velacc:  velocity accuracy (north/east in m/s)
      - navstat: navigation status
      - numsats: number of satellites tracked by primary GPS receiver
      - posmode: position mode of primary GPS receiver
    '''
    def parseOxtsData(self):
        self.oxts_dict = {}
        with open(self.oxts_file, "r") as f_in:
            for frame_id,line_string in enumerate(f_in.readlines()):
                line = line_string.strip().split(' ')
                lat = float(line[0])
                lon = float(line[1])
                yaw = float(line[5])
                self.oxts_dict[frame_id] = [lat, lon, yaw]


    
if __name__ == "__main__":

    p1 = DataParser("./data/0000.txt")
    p1.parse()
    print(p1.result_dict[10])
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0000.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([1,2,3,4,5])
    print(ccer.converted_coord_dict)
    
    

