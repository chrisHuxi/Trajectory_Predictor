import numpy as np


class KalmanFilter:
    def __init__(self, x_in, P_in, H_in, R_in, Q_in):
        self.x_ = x_in
        self.P_ = P_in

        self.H_ = H_in
        self.R_ = R_in
        self.Q_ = Q_in
        
        self.S_ = None
        self.y_ = None

    def CV_motion_model(self, t):
        
        #w_ = np.random.normal(np.mat([0.0, 0.0, 0.0, 0.0]), self.Q_, 1)
        F_CV = np.mat(  [[ 1, t, 0, 0 ],
                         [ 0, 1, 0, 0 ],
                         [ 0, 0, 1, t ],
                         [ 0, 0, 0, 1 ]])
                         
        G_CV = np.mat(  [[ t*t/2.0, 0,      0,       0 ],
                         [ 0,       t,      0,       0 ],
                         [ 0 ,      0,      t*t/2.0, 0 ],
                         [ 0,       0,      0,       t ]])
        self.x_ = F_CV * self.x_ + 0 
         
        F_CV_t = F_CV.transpose()
        G_CV_t = G_CV.transpose()       
        
        self.P_ = F_CV * self.P_ * F_CV_t + G_CV * self.Q_ * G_CV_t  

        
    def CTA_motion_model(self, acc, t):
        #w_ = np.random.normal(np.mat([0.0, 0.0, 0.0, 0.0]), self.Q_, 1)
        
        F_CTA = np.mat( [[ 1, t, 0, 0 ],
                          [ 0, 1, 0, 0 ],
                          [ 0, 0, 1, t ],
                          [ 0, 0, 0, 1 ]])
                         
        G_CTA = np.mat(  [[ t*t/2.0, 0,      0,       0 ],
                         [ 0,       t,      0,       0 ],
                         [ 0 ,      0,      t*t/2.0, 0 ],
                         [ 0,       0,      0,       t ]])  
        x_v = self.x_[1,0] + 0.0000001
        y_v = self.x_[3,0] + 0.0000001
        u = acc /np.sqrt( x_v**2 + y_v**2 ) * np.mat( [ [0],[x_v], [0], [y_v]] )
            
        self.x_ = F_CTA * self.x_ + G_CTA * u
            
        F_CTA_t = F_CTA.transpose()
        G_CTA_t = G_CTA.transpose()
            
        self.P_ = F_CTA * self.P_ * F_CTA_t + G_CTA * self.Q_ * G_CTA_t          
        

    def CT_motion_model(self, rad, t):
        #w_ = np.random.normal(np.mat([0.0, 0.0, 0.0, 0.0]), self.Q_, 1)
        F_CT = np.mat(  [[ 1, np.sin(rad * t)/rad,          0, (1 - np.cos(rad * t))/rad ],
                         [ 0, np.cos(rad * t),             0, -np.sin(rad * t)           ],
                         [ 0, (1 - np.cos(rad * t))/rad,    1, np.sin(rad * t)/rad       ],
                         [ 0, np.sin(rad * t),              0, np.cos(rad * t)           ]])
                         
        G_CT = np.mat(  [[ t*t/2.0, 0,      0,       0 ],
                         [ 0,       t,      0,       0 ],
                         [ 0 ,      0,      t*t/2.0, 0 ],
                         [ 0,       0,      0,       t ]])    

     
        self.x_ = F_CT * self.x_ + 0
            
        F_CT_t = F_CT.transpose()
        G_CT_t = G_CT.transpose()
            
        self.P_ = F_CT * self.P_ * F_CT_t + G_CT * self.Q_ * G_CT_t  
        
        
    def predict(self, motion_model_function, t):
        if motion_model_function == "CV":
            self.CV_motion_model(t)
            
        elif motion_model_function == "CTA_a1":
            self.CTA_motion_model(33.0, t)
        elif motion_model_function == "CTA_a2":
            self.CTA_motion_model(67.0, t)
        elif motion_model_function == "CTA_d1":    
            self.CTA_motion_model(-33.0, t)
        elif motion_model_function == "CTA_d2":    
            self.CTA_motion_model(-67.0, t)
            
        elif motion_model_function == "CT_l1":
            rad = 7/180.0 * np.pi
            self.CT_motion_model(rad, t)
        elif motion_model_function == "CT_r1":
            rad = -7/180.0 * np.pi
            self.CT_motion_model(rad, t)            
        elif motion_model_function == "CT_l2":
            rad = 14/180.0 * np.pi
            self.CT_motion_model(rad, t)              
        elif motion_model_function == "CT_r2":
            rad = -14/180.0 * np.pi
            self.CT_motion_model(rad, t)  

            
    def update(self, z):#z 为 measurement
        z_pred = self.H_ * self.x_; # 2*1 = 2*4 and 4 * 1
        y = z - z_pred; 
        self.y_ = y

        Ht = self.H_.transpose(); # 2*4 => 4*2

        S = self.H_ * self.P_ * Ht + self.R_; # 2*4  4*4 4*2 + 2*2
        Si = np.linalg.inv(S); 
        PHt = self.P_ * Ht;
        K = PHt * Si; 
        Kt = K.transpose()
        #print("===========")

        #print(self.x_)
        # new estimate
        self.x_ = self.x_ + (K * y);
        #print(self.x_)
        #print("===========")
        self.P_ = self.P_ - K * S * Kt 
        self.S_ = S
    
if __name__ == "__main__":
    z_dummy = np.mat(   [[1.0, 2.0],
                        [1.0, 3.0],
                        [1.0, 4.0],
                        [1.0, 5.0],
                        [1.0, 6.0],
                        [1.0, 7.0],
                        [1.0, 8.0],
                        [1.0, 9.0],
                        [1.0, 10.0],
                        [1.0, 11.0]])
    mode = 'CV'              
    x_in = np.mat([[0.0, 0.0, 0.0, 0.0]]).transpose()
    P_in = np.mat(  [[100.0,   0,     0,     0    ],
                     [0,     100.0, 0,     0    ],
                     [0,     0,     100.0,   0    ],
                     [0,     0,     0,     100.0]])
    
    
    H_in = np.mat([[1,0,0,0], [0,0,1,0]])
    R_in = np.mat([[0.1, 0],[0, 0.1]])
        
    t = 1    
    t_2 = t*t
    t_3 = t_2 * t
    t_4 = t_3 * t
    
    alpha_ax_2 = 2.25 * 2.25
    alpha_ay_2 = 2.25 * 2.25
    
    Q_in = np.mat(  [[t_4/4 * alpha_ax_2,   t_3/2 * alpha_ax_2,     0,                     0                 ],
                     [t_3/2 * alpha_ax_2,   t_2 * alpha_ax_2,       0,                     0                 ],
                     [0,                    0,                      t_4/4 * alpha_ay_2,    t_3/2 * alpha_ay_2],
                     [0,                    0,                      t_3/2 * alpha_ay_2,    t_2 * alpha_ay_2  ]]) 
                     
                     
    kf1 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
    
    kf1.predict('CT_l1',t)
    kf1.update(z_dummy[0].transpose())
    print(kf1.x_)
    
    kf1.predict('CT_l1',t)
    
    kf1.update(z_dummy[1].transpose())
    print(kf1.x_)
    
    kf1.predict('CT_l1',t)
    kf1.update(z_dummy[2].transpose())
    print(kf1.x_)    