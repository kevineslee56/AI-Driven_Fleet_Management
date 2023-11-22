import matplotlib.pyplot as plt
from matplotlib.widgets import Button

done_adding = False

def DoneAdding(event):
    done_adding = True
    print('done!')

def AddDeliveryPoints():
    del_points = plt.ginput(1)
    axcut = plt.axes([0.9, 0.0, 0.1, 0.075])
    bcut = Button(axcut, 'Done', color='red', hovercolor='green')

    # while(not done_adding):
    #     del_points.append(plt.ginput(1))

    
    bcut.on_clicked(DoneAdding)
    plt.show()
    #show on graph

