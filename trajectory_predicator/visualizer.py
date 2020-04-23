from coord_coverter import CoordCoverter
from data_parser import  DataParser
from predictor import Predictor
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.transforms as mtransforms
import matplotlib.cbook as cbook

import scipy.stats as st

#https://codeday.me/bug/20190409/919102.html
'''

    parse dict:
                frame_id = int(line[0]):
                
                track_id = int(line[1])
                object_type = line[2]
                shape = [float(line[11]),float(line[12])] #width, length (in meters)
                pose = [float(line[13]),float(line[15]),float(line[16])]  x, z, rotation_y[-pi~pi], Camera coord: x = right, y = down, z = forward, rotation = 0 when car directs to right
                
                careful!!!!!!!!!!!!

                rotation = 0 when car directs to right, rotation = -pi/2 when car directs to forward
                so we need rotation =  - (rotation_from_data - np.pi/2) + np.pi
'''
class Visualizer:
    def __init__(self, parse_dict):
        self.parse_dict = parse_dict
        
    def get_image(self, track_id):
        #fn = cbook.get_sample_data("necked_tensile_specimen.png")
        color_id = track_id%5
        if(color_id == 0):
            arr = plt.imread("car_black.png")
        elif(color_id == 1):
            arr = plt.imread("car_gray.png")         
        elif(color_id == 2):
            arr = plt.imread("car_green.png")
        elif(color_id == 3):
            arr = plt.imread("car_red.png")
        elif(color_id == 4):
            arr = plt.imread("car_white.png")            
        # make background transparent
        # you won't have to do this if your car image already has a transparent background
        #mask = (arr == (1,1,1,1)).all(axis=-1)
        #arr[mask] = 0
        return arr
 
    def imshow_affine(self, ax, z, *args, **kwargs):
        im = ax.imshow(z, *args, **kwargs)
        x1, x2, y1, y2 = im.get_extent()
        im._image_skew_coordinate = (x2, y1)
        return im
        
    def create_image_for_1_frame(self, frame_id):
    
        #fig, ax = plt.subplots(figsize=(10,8))
        fig, ax = plt.subplots(figsize=(8,8))
        for track_id in set(self.parse_dict[frame_id].keys()):
            #track_id = object_info[0]
            x = self.parse_dict[frame_id][track_id][2][0]
            y = self.parse_dict[frame_id][track_id][2][1]
            rotation = - (self.parse_dict[frame_id][track_id][2][2] - np.pi/2) + np.pi
            
            width = self.parse_dict[frame_id][track_id][1][0]
            length = self.parse_dict[frame_id][track_id][1][1]
            #x, y, speed, heading = s[:4]
            #car_xmin = x - self.REAR_WHEEL_RELATIVE_LOC
            object_x_min = x - length/2.0
            #car_ymin = y - self.CAR_WIDTH / 2.
            object_y_min = y - width/2.0
            
            car_fig = self.get_image(track_id)

            im = self.imshow_affine(ax, car_fig, interpolation='none',
                       extent=[0, width, 0, length], clip_on=True)
            center_x, center_y = width//2, length//2
            im_trans = (mtransforms.Affine2D()
                .rotate_around(center_x, center_y, rotation)
                .translate(x, y)
                + ax.transData)
                
            im.set_transform(im_trans)           
        
        
        xmin, xmax = -30, 30
        ymin, ymax = 0, 110
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        
        plt.savefig("./vis_result/"+"frame"+ str(frame_id) +".png")
        #plt.show()
        fig.clf()
        ax.cla()
        
    def create_image_for_all_frames(self):
        for frame_id in self.parse_dict.keys():
            self.create_image_for_1_frame(frame_id)
    
    
    # https://towardsdatascience.com/simple-example-of-2d-density-plots-in-python-83b83b934f67
    # https://stackoverflow.com/questions/30145957/plotting-2d-kernel-density-estimation-with-python
    def create_image_for_1_frame_predict(self, frame_id, predict_dict, track_ids):
        fig, ax = plt.subplots(figsize=(8,8))
        xmin, xmax = -30, 30
        ymin, ymax = 0, 110
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        
        for track_id in track_ids:
            try:
                              
                # ================= gaussian density ================ #
                pose_x_estimate = predict_dict[track_id][0][0,0]
                pose_vx_estimate = predict_dict[track_id][0][1,0]
                
                pose_y_estimate = predict_dict[track_id][0][2,0]
                pose_vy_estimate = predict_dict[track_id][0][3,0]

                covariance_mat = [  [predict_dict[track_id][1][0,0],predict_dict[track_id][1][0,2]],
                                    [predict_dict[track_id][1][2,0],predict_dict[track_id][1][2,2]]    ]
                dummy_data = np.random.multivariate_normal((pose_x_estimate, pose_y_estimate), covariance_mat, 500)
                
                x_gaussian = dummy_data[:, 0]
                y_gaussian = dummy_data[:, 1]
                # Peform the kernel density estimate
                xx, yy = np.mgrid[xmin:xmax:300j, ymin:ymax:300j]
                positions = np.vstack([xx.ravel(), yy.ravel()])
                values = np.vstack([x_gaussian, y_gaussian])
                kernel = st.gaussian_kde(values)
                f = np.reshape(kernel(positions).T, xx.shape)
                print(np.amax(f))
                #print(f)
                # Contourf plot
                #step = 0.0005
                m = np.amax(f)
                levels = np.arange(0.000001,m, m/5.0)
                plt.contourf(xx, yy, f, 5, cmap=plt.cm.coolwarm, alpha=0.5, levels = levels)
                ax.scatter(pose_x_estimate,pose_y_estimate, color="red", linewidth=1, edgecolor="ivory", s=50, label='predicted')
                
                plt.annotate(s='', xy=(pose_x_estimate + pose_vx_estimate, pose_y_estimate + pose_vy_estimate), 
                            xytext=(pose_x_estimate,pose_y_estimate), 
                            arrowprops=dict(arrowstyle='-|>',color='red',linewidth=3))
                # ================= gaussian density: test done ================ #

                
                #track_id = object_info[0]
                x = self.parse_dict[frame_id][track_id][2][0]
                y = self.parse_dict[frame_id][track_id][2][1]
                groundtruth_dot = ax.scatter(x,y, color="blue", linewidth=1, edgecolor="ivory", s=50, label='ground truth')
                plt.legend(loc="upper right")
            except:
                print("there is no track_id in this frame or predict_dict!!")
                

        plt.savefig("./vis_result/"+"frame_predict_"+ str(frame_id)+ "_obj_" + str(track_id) +".png")
        #plt.show()
        fig.clf()
        ax.cla()  

def plot_RSME(xy_camera, xy_lidar):
    # multiple line plot
    x_camera, y_camera = xy_camera
    x_lidar, y_lidar = xy_lidar
    plt.plot( x_camera, y_camera, marker='', color='olive', linewidth=2, label="camera RSME")
    plt.plot( x_lidar, y_lidar, marker='', color='red', linewidth=2, linestyle='dashed', label="lidar RSME")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    '''
    p1 = DataParser("./data/0000.txt")
    p1.parse()
    '''
    '''
    v1 = Visualizer(p1.result_dict)
    v1.create_image_for_1_frame(0)
    v1.create_image_for_1_frame(1)
    v1.create_image_for_1_frame(2)
    v1.create_image_for_1_frame(3)
    v1.create_image_for_1_frame(4)
    v1.create_image_for_1_frame(5)
    v1.create_image_for_1_frame(6)
    v1.create_image_for_1_frame(7)
    v1.create_image_for_1_frame(8)
    '''
    '''
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0000.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([0,1,2,3,4,5,6,7,8])
    v1 = Visualizer(ccer.converted_coord_dict)
    v1.create_image_for_1_frame(0)
    v1.create_image_for_1_frame(1)
    v1.create_image_for_1_frame(2)
    v1.create_image_for_1_frame(3)
    v1.create_image_for_1_frame(4)
    v1.create_image_for_1_frame(5)
    v1.create_image_for_1_frame(6)
    v1.create_image_for_1_frame(7)
    v1.create_image_for_1_frame(8)
    '''

    p1 = DataParser("./data/0002.txt",'lidar')
    p1.parse()
    #p2 = DataParser("./data/0.txt",'mono-camera')
    #p2.parse()    
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0002.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([ i for i in range(60)])
    print(ccer.converted_coord_dict.keys())

    v1 = Visualizer(ccer.converted_coord_dict)
    
    predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 10, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 0.0)    
    v1.create_image_for_1_frame_predict(10, predictor.predict_dict, [846])
    
    predictor.predict(period_of_predict = 0.5)    
    v1.create_image_for_1_frame_predict(15, predictor.predict_dict, [846])
    #v1.create_image_for_1_frame_evaluate(self, frame_id_ground_truth = 15, predict_dict, track_ids)
    
    #predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 15, period_of_predict = 0.5, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 1.0)
    v1.create_image_for_1_frame_predict(20, predictor.predict_dict, [846])
    #v1.create_image_for_1_frame_evaluate(self, frame_id_ground_truth = 15, predict_dict, track_ids)

    predictor.predict(period_of_predict = 1.5)
    v1.create_image_for_1_frame_predict(25, predictor.predict_dict, [846])
    
    predictor.predict(period_of_predict = 2.0)
    v1.create_image_for_1_frame_predict(30, predictor.predict_dict, [846])
    
    predictor.predict(period_of_predict = 2.5)
    v1.create_image_for_1_frame_predict(35, predictor.predict_dict, [846])
    
    #v1.create_image_for_1_frame(15)
    #v1.create_image_for_all_frames()

    
    
    
    
    
    
    
    
    
    