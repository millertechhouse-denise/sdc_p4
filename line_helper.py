import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class Prev_Lines():
    """
    Class to keep last n Lines
    Circular List implementation
    
    """
    def __init__(self):
        self.line_list = []
        self.index = 0
        self.full_list = False
        self.count = 8
        
    def add_line(self, ln):
        """
        Adds a Line to the list
    
        """
        if self.full_list == False:
            self.line_list.append(ln)
            self.index = (self.index + 1) 
            if self.index >= self.count:
                self.full_list = True
                self.index = self.index  % self.count 
            
        else:
            self.line_list[self.index] = ln
            self.index = (self.index + 1) % self.count
        
            
    def get_line_data(self):
        """
        Calculates the Line data given all line data in list
    
        """
        new_line = Line()
        
        for ln in self.line_list :
            new_line.allx = new_line.allx +ln.recent_xfitted
            new_line.best_fit = new_line.best_fit + ln.current_fit
            new_line.ally = ln.ally
        if self.full_list == True:
            new_line.best_fit = new_line.best_fit / self.count
            new_line.allx = new_line.allx / self.count
        else:
            new_line.best_fit = new_line.best_fit / self.index
            new_line.allx = new_line.allx / self.index
            
        return new_line
        
               
# Define a class to receive the characteristics of each line detection
class Line():
    """
    Class to hold characteristics for line detection
    
    """
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = np.array([0,0,0], dtype='float') 
        #polynomial coefficients for the most recent fit
        self.current_fit = np.array([0,0,0], dtype='float') 
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = np.zeros(720)
        #y values for detected line pixels
        self.ally = None
