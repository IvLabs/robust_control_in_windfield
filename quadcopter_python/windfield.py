from math import ceil
import numpy as np


class Wind:
    def __init__(self):
        self.grid_len = 6
        self.mean = 0
        self.std_dev = 1
        self.noise_factor = 0.6
        self.discount_factor = 0.9
        x = np.arange(-self.grid_len, self.grid_len, 0.4)
        y = np.arange(-self.grid_len, self.grid_len, 0.4)
        #num_samples = len(x)**2
        self.X, self.Y = np.meshgrid(x,y)
        self.oldnoise_x = self.currnoise_x = np.zeros((len(x),len(x)))
        self.oldnoise_y = self.currnoise_y = np.zeros((len(x),len(x)))

    def xy_to_rc(self,x,y):
        c = ceil((x + self.grid_len)/0.4) 
        r = ceil((y + self.grid_len)/0.4)
        return max(0,min(r,29)),max(0,min(c,29))



    def gaussian_noise_gen(self):

        num_samples = len(self.X)**2
        noise_x = np.reshape(np.random.normal(self.mean, self.std_dev, size=num_samples),(len(self.X),len(self.X)))
        noise_y = np.reshape(np.random.normal(self.mean, self.std_dev, size=num_samples),(len(self.X),len(self.X)))

        self.currnoise_x = self.oldnoise_x*self.discount_factor + (1 - self.discount_factor)*noise_x
        self.currnoise_y = self.oldnoise_y*self.discount_factor + (1 - self.discount_factor)*noise_y

        u = (np.sin((self.X + self.Y)/3)) + self.noise_factor*self.currnoise_x
        v = (np.cos((self.X - self.Y)/3)) + self.noise_factor*self.currnoise_x

        self.oldnoise_x = self.currnoise_x
        self.oldnoise_y = self.currnoise_y

        return u,v

    def windfield_gen(self,xcoords,ycoords):

        row,col = self.xy_to_rc(xcoords,ycoords)
        u,v = self.gaussian_noise_gen()
        wind_vector = np.array([u[row,col],v[row,col],0])
        return wind_vector,u,v
        