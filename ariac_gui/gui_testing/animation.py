import tkinter as tk
from functools import partial

RADIUS = 5
DELAY = 1000  # milliseconds
WIDTH, HEIGHT = 900, 500

def print_dict(d : dict):
    for i in range(len(d)):
        print(i, d[list(d.keys())[i]])

class gui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.num_labels = 3
        self.coordinates = [(i,100) for i in range(0,WIDTH,1)]

        self.current_index = 0
        self.current_index_dict = {}
        self.label_index = 0

        self.canvas = tk.Canvas(self, width=WIDTH, height=HEIGHT)
        self.canvas.pack()

        self.labels = [tk.Label(self,text=str(i)) for i in range(1000)]
        self.current_index_dict={label:0 for label in self.labels}
        self.current_labels = [(self.canvas.create_window(self.coordinates[self.current_index_dict[self.labels[self.label_index]]],window=self.labels[self.label_index]),self.labels[self.label_index])]

        self.current_index_dict[self.labels[self.label_index]] +=1

        self.canvas.after(DELAY, self.move)

    def move(self):
        list_changed = False
        for i in range(len(self.current_labels)):
            loop_i = i if not list_changed else i-1
            self.canvas.coords(self.current_labels[loop_i][0], self.coordinates[self.current_index_dict[self.current_labels[loop_i][1]]])
            self.current_index_dict[self.current_labels[loop_i][1]]+=1
            current_index = self.current_index_dict[self.current_labels[loop_i][1]]
            if current_index==len(self.coordinates)//self.num_labels:
                self.label_index=(self.label_index+1)%len(self.labels)
                self.current_labels.append((self.canvas.create_window(self.coordinates[0], window=self.labels[self.label_index]),self.labels[self.label_index]))
            if current_index>=len(self.coordinates):
                list_changed = True
                self.current_index_dict[self.current_labels[loop_i][1]] = 0
                self.canvas.delete(self.current_labels[0][0])
                del self.current_labels[0]
        self.canvas.after(DELAY, self.move)

if __name__=="__main__":
    app = gui()
    app.mainloop()