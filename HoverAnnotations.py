import matplotlib.pyplot as plt
import numpy as np; np.random.seed(1)

class HoverAnnotation:
    def __init__(self, line_collection, annotation, names, ax, fig) -> None:
        self.lines = line_collection
        self.annotation = annotation
        self.names = names
        self.ax = ax
        self.fig = fig
        
    def update_annot(self, ind):
        x,y = self.lines.get_data()
        annot = self.annotation
        annot.xy = (x[ind["ind"][0]], y[ind["ind"][0]])
        text = "{}, {}".format(" ".join(list(map(str,ind["ind"]))), 
                            " ".join([self.names[n] for n in ind["ind"]]))
        annot.set_text(text)
        annot.get_bbox_patch().set_alpha(0.4)

    def hover(self, event):
        vis = self.annot.get_visible()
        if event.inaxes == self.ax:
            cont, ind = self.lines.contains(event)
            if cont:
                self.update_annot(ind)
                self.annot.set_visible(True)
                self.fig.canvas.draw_idle()
            else:
                if vis:
                    self.annot.set_visible(False)
                    self.fig.canvas.draw_idle()
