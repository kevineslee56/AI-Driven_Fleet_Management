import matplotlib.pyplot as plt
import numpy as np; np.random.seed(1)

class HoverAnnotation:
    def __init__(self, path_info, ax, fig):
        self.path_info = path_info
        self.ax = ax
        self.fig = fig

        self.hover_annot = ax.annotate("", xy=(0,0), xytext=(20,20),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc=(1,1,1,1), ec=(0,0,0,1)),
                    arrowprops=dict(arrowstyle="->"),zorder=20)
        self.hover_annot.set_visible(False)

        self.click_annot = ax.annotate("", xy=(0,0), xytext=(20,20),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc=(1,1,1,1), ec=(0,0,0,1)),
                    arrowprops=dict(arrowstyle="->"),zorder=20)
        self.click_annot.set_visible(False)
        
    def update_node_annot(self, ind, i):
        idx = ind["ind"][0]
        sc = self.path_info[i][4]
        pos = sc.get_offsets()[idx]
        node_x = sc.get_offsets().data[idx][0]
        node_y = sc.get_offsets().data[idx][1]
        sequence_distance = self.path_info[i][3][idx]

        annot = self.hover_annot
        annot.xy = pos 
        text = "(x, y): ({}, {})\n".format(int(node_x), int(node_y)) + "Path Cost: " + f"{sequence_distance:.2f}"  + "\npackage #" + str(ind["ind"][0]+1)
        annot.set_text(text)
        annot.get_bbox_patch().set_facecolor('w')
        annot.get_bbox_patch().set_edgecolor('black')
        annot.get_bbox_patch().set_alpha(1)

    def update_line_annot(self, line, x, y):
        annot = self.click_annot
        annot.xy = (x, y)
        for i, path_info in enumerate(self.path_info):
            if line == path_info[0]:
                text = "Route length: " + f"{path_info[1]:.2f}" + "\n#Deliveries: " + str(path_info[2])
                annot.set_text(text)
                annot.get_bbox_patch().set_facecolor('w')
                annot.get_bbox_patch().set_edgecolor('black')
                annot.get_bbox_patch().set_alpha(1)

    def hover(self, event):
        vis = self.hover_annot.get_visible()
        for i, sc in enumerate([l[4] for l in self.path_info]):
            cont, ind = sc.contains(event)
            if cont:
                self.update_node_annot(ind, i)
                self.hover_annot.set_visible(True)
                self.click_annot.set_visible(False)
                self.fig.canvas.draw_idle()

    def click(self, event):
        vis = self.click_annot.get_visible()
        for line in [l[0] for l in self.path_info]:
            cont, ind = line.contains(event)
            if cont:
                self.update_line_annot(line, event.xdata, event.ydata)
                self.click_annot.set_visible(True)
                self.hover_annot.set_visible(False)
                self.fig.canvas.draw_idle()
            else:
                if vis:
                    self.click_annot.set_visible(False)
                    self.fig.canvas.draw_idle()
    
    def test_callback(event):
        print("asdf")
