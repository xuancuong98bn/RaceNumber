import numpy as np

class Line():
    def __init__(self):
        
        # was the line detected in the last iteration?
        self.detected = False  
        
        # polynomial coefficients averaged over the last n iterations
        self.best_fit_equation = []
        self.best_fit_m = None
        
        #polynomial coefficients for the most recent fit
        self.current_lane_equation = []  
        self.current_fit_m = None
        self.check_point = []

        #radius of curvature of the line in some units
        self.radius_of_curvature = None
        
        # center position of car
        self.lane_to_camera = None
        
        # Previous Fits
        self.previous_lane_equation = []
        self.previous_fits_m = []
        
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float')
        
        # meters per pixel in y dimension
        self.ym_per_pix = 30/720
        
        # y_eval is where we want to evaluate the fits for the line radius calcuation 
        # for us it's at the bottom of the image for us, and because we know 
        # the size of our video/images we can just hardcode it
        self.y_eval = 720. * self.ym_per_pix
        
        # camera position is where the camera is located relative to the image
        # we're assuming it's in the middle
        self.camera_position = 155.

    def run_line_pipe(self):
        self.calc_best_fit()
        
    def __get_line__(self):
        return self.best_fit_equation, self.check_point

    def __add_new_fit__(self, lane_equation, new_fit_m, check_point):
        """
        Add a new fit to the Line class
        """
        self.check_point = check_point
        self.best_fit_equation = lane_equation
        # print(lane_equation)
        # If this is our first line, then we will have to take it
        # if len(self.current_lane_equation) == 0 and len(self.previous_lane_equation) == 0:
        #     self.detected = True
        #     self.current_lane_equation = lane_equation
        #     self.check_horizontal()
        #     self.previous_lane_equation = self.current_lane_equation
        #     return
        # else:
        #     self.current_lane_equation = lane_equation
        #     self.run_line_pipe()
        #     return

    def check_vertical(self):
        print("ec")
        print(self.current_lane_equation)

        print(self.previous_lane_equation)
        for i in range(len(self.current_lane_equation)):
            self.diffs = np.abs(self.current_lane_equation[i] - self.previous_lane_equation[i])
            if not self.diff_check():
                self.current_lane_equation[i] = self.previous_lane_equation[i]

    def diff_check(self):
        if self.diffs[0] > 10:
            return False
        if self.diffs[1] > 50:
            return False
        return True

    def check_horizontal(self):
        a = [] ; b = [] ; result = []
        for i in self.current_lane_equation:
            a.append(i[0])
            b.append(i[1])
        result.append([a[0],b[0]])
        for i in range(1,len(a)-1):
            prev_diff = np.abs(a[i]-a[i-1])
            next_diff = np.abs(a[i]-a[i+1])
            if prev_diff > 10 and next_diff > 10:
                a[i]=(a[i-1]+a[i+1])/2
                b[i]=(b[i-1]+b[i+1])/2
            result.append([a[i],b[i]])
        max = len(a)-1
        result.append([a[max],b[max]])
        self.current_lane_equation = result
        self.best_fit_equation = self.current_lane_equation

    def calc_best_fit(self):
        self.check_vertical()
        self.check_horizontal()
        self.previous_lane_equation = self.best_fit_equation
        return
        
    def calc_radius(self):
        return