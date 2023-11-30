import matplotlib.pyplot as plt
import numpy as np; np.random.seed(1)

class HoverAnnotation:
    def __init__(self, sc, lines, distances, deliveries, annotation, ax, fig, x, y):
        self.sc = sc
        self.lines = lines
        self.distances = distances
        self.deliveries = deliveries
        self.annotation = annotation
        self.ax = ax
        self.fig = fig
        self.x = x
        self.y = y
        
    def update_annot(self, ind):
        pos = self.sc.get_offsets()[ind["ind"][0]]
        annot = self.annotation
        annot.xy = pos 
        text = "x = {}\ny= {}".format(self.x[ind["ind"][0]], self.y[ind["ind"][0]]) + "\nPath Cost:" #+ f"{self.distances[i]:.2f}"
        annot.set_text(text)
        annot.get_bbox_patch().set_facecolor('w')
        annot.get_bbox_patch().set_edgecolor('black')
        annot.get_bbox_patch().set_alpha(1)

    def update_line_annot(self, line, x, y):
        annot = self.annotation
        annot.xy = (x, y)
        for i, l in enumerate(self.lines):
            if line == l:
                text = "Route length: " + f"{self.distances[i]:.2f}" + "\nDeliveries: " + str(self.deliveries[i])
                annot.set_text(text)
                annot.get_bbox_patch().set_facecolor('w')
                annot.get_bbox_patch().set_edgecolor('black')
                annot.get_bbox_patch().set_alpha(1)

    def hover(self, event):
        vis = self.annotation.get_visible()
        if event.inaxes == self.ax:
            cont, ind = self.sc.contains(event)
            if cont:
                self.update_annot(ind)
                self.annotation.set_visible(True)
                self.fig.canvas.draw_idle()
            else:
                if vis:
                    self.annotation.set_visible(False)
                    self.fig.canvas.draw_idle()

    def click(self, event):
        vis = self.annotation.get_visible()
        for line in self.lines:
                cont, ind = line.contains(event)
                if cont:
                    self.update_line_annot(line, event.xdata, event.ydata)
                    self.annotation.set_visible(True)
                    self.fig.canvas.draw_idle()
                else:
                    if vis:
                        self.annotation.set_visible(False)
                        self.fig.canvas.draw_idle()
    
    def test_callback(event):
        print("asdf")
