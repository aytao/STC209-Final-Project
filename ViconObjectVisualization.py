'''--------------------------------------------------------------------------
Title: STC 209 - Spring 2023: Track movement of hand holder in the StudioLab
Author: Maria Santos, maria.santos@princeton.edu
Description: This script plots the XZ position of an object.
The Y coordinate gives the size of the marker
Date: March 7, 2023
-------------------------------------------------------------------------'''

import matplotlib.pyplot as plt
import numpy as np

class MarkersVisualization:

    def __init__(self, domain, vicon_objects):
        self.figsize = np.array([12, 10])
        self.fontsize = 16
        self.linewidth = 4
        self.markersize = 10000
        
        self._fig, self._axes = self.__init_figure(domain)
        self.__draw_domain(domain)
        self._hvicon_objects = self.__init_objects(vicon_objects)


    def update_objects(self, objects):
        self._hvicon_objects.set_offsets(np.c_[objects[:,1], objects[:,2]])
        self._hvicon_objects.set_sizes([self.markersize - objects[:,0]*10000])
        self.redraw_figure()

    def redraw_figure(self):
        self._fig.canvas.flush_events()
        plt.show(block=False)


    def __init_figure(self, domain):
        plt.ion()
        figsize = self.figsize#np.abs(self.rotate(np.transpose(self.figsize)))
        fig, axes = plt.subplots(figsize=(np.int(figsize[0]), figsize[1]))
        # domain = self.rotate(domain)
        plt.xlim(1.1*np.min(domain[:,0]), 1.1*np.max(domain[:,0]))
        plt.ylim(1.1*np.min(domain[:,1]), 1.1*np.max(domain[:,1]))
        axes.axis('equal')

        #set fontsizes
        plt.rcParams['font.size'] = self.fontsize
        plt.rcParams['figure.autolayout'] = True
        plt.xticks(fontsize=0.8*self.fontsize)
        plt.yticks(fontsize=0.8*self.fontsize)
        axes.set_xlabel('y [m]', fontsize=self.fontsize)
        axes.set_ylabel('z [m]', fontsize=self.fontsize)

        plt.draw()
        
        return fig, axes

    def __draw_domain(self, domain):
        # domain = self.rotate(domain)
        self._axes.plot(domain[:,0], domain[:,1], \
                        linestyle='-', linewidth=self.linewidth, color='black')
        self.redraw_figure()


    def __init_objects(self, robots):
        # robots = self.rotate(robots)
        hvicon_objects = self._axes.scatter(robots[:,0], robots[:,1], s=self.markersize)
        self.redraw_figure()
        return hvicon_objects


        
