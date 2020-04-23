import time

import numpy as np

from coord_coverter import CoordCoverter
from data_parser import  DataParser
from predictor import Predictor

from visualizer import plot_RSME

class Evaluator:
    def __init__(self):
        pass
          
    # we don't have velocity in ground truth data
    def evalate(self, ground_truth_pose_dict, predicted_pose_dict, frame_id, track_id):

        x_gt = ground_truth_pose_dict[frame_id][track_id][2][0]
        y_gt = ground_truth_pose_dict[frame_id][track_id][2][1]
        
        x_estimate = predicted_pose_dict[track_id][0][0,0]
                
        y_estimate = predicted_pose_dict[track_id][0][2,0]
     
        RSME_position = np.sqrt( (x_gt - x_estimate) * (x_gt - x_estimate) + (y_gt - y_estimate) * (y_gt - y_estimate) )
        #RSME_velocity = ()
        return RSME_position
        
    # we don't have velocity in ground truth data
    def evalate_with_groundtruth(self, ground_truth_pose_dict, predicted_pose_dict, frame_id, track_id_gt, track_id_pred):

        x_gt = ground_truth_pose_dict[frame_id][track_id_gt][2][0]
        y_gt = ground_truth_pose_dict[frame_id][track_id_gt][2][1]

        x_estimate = predicted_pose_dict[track_id_pred][0][0,0]
                
        y_estimate = predicted_pose_dict[track_id_pred][0][2,0]
        
        print('------------')
        print(x_gt)
        print(y_gt)
        print(x_estimate)
        print(y_estimate)
        
        
        print('------------')
        
        RSME_position = np.sqrt( (x_gt - x_estimate) * (x_gt - x_estimate) + (y_gt - y_estimate) * (y_gt - y_estimate) )
        #RSME_velocity = ()
        return RSME_position


if __name__ == "__main__":
    '''
    p_gt = DataParser("./data/0002.txt",'lidar')
    p_gt.parse()  
    gt_ccer = CoordCoverter(p_gt.result_dict, "./data_tracking_oxts/0002.txt")
    gt_ccer.setCoordOrigin(0)
    gt_ccer.convertCoord([ i for i in range(200)])
    
    
    p1 = DataParser("./data/2.txt",'mono-camera')
    p1.parse()  

    
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0002.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([ i for i in range(200)])

    predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 155, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 0.0) 
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = 155, track_id_gt=1036, track_id_pred = 155))
    
    frame_id_list = [i for i in range(155,180)]
    
    # for plot: 
    x_camera = []
    y_camera = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        predictor.predict(period_of_predict = 0.1 * index)  
        x_camera.append(index)
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = frame_id, track_id_gt= 1036, track_id_pred = 155)
        print(RSME_tmp)
        y_camera.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)

    
    
    lidar_predictor = Predictor(gt_ccer.converted_coord_dict, current_frame_id = 155, predictor_mode = "AMM")
    lidar_predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = 155, track_id_gt=1036, track_id_pred = 1036))
    
    frame_id_list = [i for i in range(155,180)]

    x_lidar = []
    y_lidar = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        lidar_predictor.predict(period_of_predict = 0.1 * index)  
        x_lidar.append(index)   
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = frame_id, track_id_gt= 1036, track_id_pred = 1036)
        print(RSME_tmp)
        y_lidar.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)
    
    plot_RSME((x_camera, y_camera), (x_lidar, y_lidar))
    '''
    
    
    '''
    p_gt = DataParser("./data/0002.txt",'lidar')
    p_gt.parse()  
    gt_ccer = CoordCoverter(p_gt.result_dict, "./data_tracking_oxts/0002.txt")
    gt_ccer.setCoordOrigin(0)
    gt_ccer.convertCoord([ i for i in range(100)])

    
    p1 = DataParser("./data/2.txt",'mono-camera')
    p1.parse()  
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0002.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([ i for i in range(100)])

    
    predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 25, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = 25, track_id_gt=846, track_id_pred = 147))
    
    frame_id_list = [i for i in range(25,50)]
    
    # for plot: 
    x_camera = []
    y_camera = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        predictor.predict(period_of_predict = 0.1 * index)  
        x_camera.append(index)
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = frame_id, track_id_gt= 846, track_id_pred = 147)
        print(RSME_tmp)
        y_camera.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)

    
    
    lidar_predictor = Predictor(gt_ccer.converted_coord_dict, current_frame_id = 10, predictor_mode = "AMM")
    lidar_predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = 10, track_id_gt=846, track_id_pred = 846))
    
    frame_id_list = [i for i in range(10,40)]

    x_lidar = []
    y_lidar = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        lidar_predictor.predict(period_of_predict = 0.1 * index)  
        x_lidar.append(index)   
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = frame_id, track_id_gt= 846, track_id_pred = 846)
        print(RSME_tmp)
        y_lidar.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)
    
    plot_RSME((x_camera, y_camera), (x_lidar, y_lidar))
    #predictor.predict(period_of_predict = 0.5)
    '''
    
    
    
    
    
    
    
    
    
    
    '''
    p_gt = DataParser("./data/0003.txt",'lidar')
    p_gt.parse()  
    gt_ccer = CoordCoverter(p_gt.result_dict, "./data_tracking_oxts/0003.txt")
    gt_ccer.setCoordOrigin(0)
    gt_ccer.convertCoord([ i for i in range(100)])
    
    
    p1 = DataParser("./data/3.txt",'mono-camera')
    p1.parse()  
    
    print("test")
    print(p_gt.result_dict[60][1369]) 
    print(p1.result_dict[60][184]) 
    
    print("test")
    
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0003.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([ i for i in range(100)])
    
    print('+++++++++')
    print(p1.result_dict[60])    
    print('+++++++++')
    print(p_gt.result_dict[60])    
    print('+++++++++')
    print(ccer.converted_coord_dict[60])
    print('+++++++++')
    print(gt_ccer.converted_coord_dict[60])
    print('+++++++++')
    
    predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 60, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 0.1)
    print(predictor.predict_dict)
    print('+++++++++')
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = 60, track_id_gt = 1369, track_id_pred = 184))
    
    frame_id_list = [i for i in range(60,90)]
    
    # for plot: 
    x_camera = []
    y_camera = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        predictor.predict(period_of_predict = 0.1 * index)  
        x_camera.append(index)
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = frame_id, track_id_gt= 1369, track_id_pred = 184)
        print(RSME_tmp)
        y_camera.append(RSME_tmp)
        
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)

    
    
    lidar_predictor = Predictor(gt_ccer.converted_coord_dict, current_frame_id = 60, predictor_mode = "AMM")
    lidar_predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = 60, track_id_gt = 1369, track_id_pred = 1369))
    
    frame_id_list = [i for i in range(60,90)]

    x_lidar = []
    y_lidar = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        lidar_predictor.predict(period_of_predict = 0.1 * index)  
        x_lidar.append(index)   
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = frame_id, track_id_gt= 1369, track_id_pred = 1369)
        print(RSME_tmp)
        y_lidar.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    print(RSME_mean)
    
    
    plot_RSME((x_camera, y_camera), (x_lidar, y_lidar))
    #predictor.predict(period_of_predict = 0.5)
    '''
    
    
    
    p_gt = DataParser("./data/0008.txt",'lidar')
    p_gt.parse()  
    gt_ccer = CoordCoverter(p_gt.result_dict, "./data_tracking_oxts/0008.txt")
    gt_ccer.setCoordOrigin(0)
    gt_ccer.convertCoord([ i for i in range(100)])

    
    p1 = DataParser("./data/8.txt",'mono-camera')
    p1.parse()  
    ccer = CoordCoverter(p1.result_dict, "./data_tracking_oxts/0008.txt")
    ccer.setCoordOrigin(0)
    ccer.convertCoord([ i for i in range(100)])

    print('+++++++++')
    print(p1.result_dict[10][296])    
    print('+++++++++')
    print(p_gt.result_dict[10][3702])    
    print('+++++++++')
    print(ccer.converted_coord_dict[10][296])
    print('+++++++++')
    print(gt_ccer.converted_coord_dict[10][3702])
    print('+++++++++')
    
    predictor = Predictor(ccer.converted_coord_dict, current_frame_id = 30, predictor_mode = "AMM")
    predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = 30, track_id_gt=3702, track_id_pred = 296))
    
    frame_id_list = [i for i in range(30,55)]
    
    # for plot: 
    x_camera = []
    y_camera = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        time_start=time.time()
        predictor.predict(period_of_predict = 0.1 * index)  
        time_end=time.time()
        print('time cost',time_end-time_start,'s')
        
        x_camera.append(index)
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, predictor.predict_dict, frame_id = frame_id, track_id_gt= 3702, track_id_pred = 296)
        print(RSME_tmp)
        y_camera.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)

    
    
    lidar_predictor = Predictor(gt_ccer.converted_coord_dict, current_frame_id = 30, predictor_mode = "AMM")
    lidar_predictor.predict(period_of_predict = 0.0)
    
    e1 = Evaluator()
    
    print(e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = 30, track_id_gt=3702, track_id_pred = 3702))
    
    frame_id_list = [i for i in range(30,55)]

    x_lidar = []
    y_lidar = []
    
    RSME_mean = 0.0
    for index,frame_id in enumerate(frame_id_list):
        
        lidar_predictor.predict(period_of_predict = 0.1 * index)  
        x_lidar.append(index)   
        
        RSME_tmp = e1.evalate_with_groundtruth(gt_ccer.converted_coord_dict, lidar_predictor.predict_dict, frame_id = frame_id, track_id_gt= 3702, track_id_pred = 3702)
        print(RSME_tmp)
        y_lidar.append(RSME_tmp)
        RSME_mean += RSME_tmp
            
    RSME_mean = RSME_mean/len(frame_id_list)
    
    print("===============")
    
    print(RSME_mean)
    
    plot_RSME((x_camera, y_camera), (x_lidar, y_lidar))
    #predictor.predict(period_of_predict = 0.5) 
    
    