
class DataParser:
    def __init__(self, file_name, data_type = 'lidar'):
        self.file_name = file_name
        self.result_dict = {}
        self.data_type = data_type
    def parse(self):
        self.result_dict = {}
        with open(self.file_name, "r") as f_in:
            if self.data_type == 'lidar':
                for line_string in f_in.readlines():
                    line = line_string.strip().split(' ')
                   
                    #print(line)
                    frame_id = int(line[0])
                    
                    #in case: 这一 frame 中没有 track
                    try:
                        track_id = int(line[1])
                        object_type = line[2]
                        shape = [float(line[11]),float(line[12])] #width, length (in meters)
                        pose = [float(line[13]),float(line[15]),float(line[16])] 
                        #x, z, rotation_y[-pi~pi], Camera coord: x = right, y = down, z = forward
                        #这里的坐标系是 x 向右, z 是向前, 而 y 是向下
                        #The reference point for the 3D bounding box for each object is centered on the bottom face of the box
                        #这个 x 和 z 是 3d box 的底面的中心点的 x 和 z


                        if frame_id not in set(self.result_dict.keys()):
                            self.result_dict[frame_id] = {track_id:[object_type,shape,pose]}
                        else:
                            self.result_dict[frame_id][track_id] = [object_type,shape,pose]

                    except:
                        print("empty track")
                        self.result_dict[frame_id] = {}
            elif self.data_type == 'mono-camera':
                for line_string in f_in.readlines():
                    line = line_string.strip().split(' ')
                   
                    frame_id = int(line[0])
                    
                    #in case: 这一 frame 中没有 track
                    try:
                        track_id = int(line[1])
                        object_type = 'NOT_GEIVEN'
                        shape = [float(line[2]),float(line[3])] #width, length (in meters)
                        pose = [float(line[5]),float(line[4]),float(line[6])] 
                        #x, z, rotation_y[-pi~pi], Camera coord: x = right, y = down, z = forward
                        #这里的坐标系是 x 向右, z 是向前, 而 y 是向下
                        #The reference point for the 3D bounding box for each object is centered on the bottom face of the box
                        #这个 x 和 z 是 3d box 的底面的中心点的 x 和 z
                        
                        if frame_id not in set(self.result_dict.keys()):
                            self.result_dict[frame_id] = {track_id:[object_type,shape,pose]}
                        else:
                            self.result_dict[frame_id][track_id] = [object_type,shape,pose]

                    except:
                        print("empty track")
                        self.result_dict[frame_id] = {}

if __name__ == "__main__":
    p1 = DataParser("./data/0000.txt", 'lidar')
    p1.parse()
    
    p2 = DataParser("./data/0.txt", 'mono-camera')
    p2.parse()
    print(p1.result_dict[0])
    
        