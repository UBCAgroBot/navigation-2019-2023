from tkinter import (HORIZONTAL, Button, Checkbutton, Frame, IntVar, Label,
                     Radiobutton, Tk, ttk, PhotoImage, Canvas, Image)

import cv2
import numpy as np

class SimpleGUI(Tk):

    def __init__(self, frame):
        super().__init__()
        self.frame = frame
        self.initializeUI()

    def initializeUI(self):
        self.title("Live View")
        self.minsize(300, 200)  # width, height
        self.geometry("400x350+50+50")
        self.setupWindow()

    def setupWindow(self):
        """ Set up the widgets."""
        title = Label(self, text="Live Toggles",
                      font=('Helvetica', 20), bd=10)
        title.pack()

        line = ttk.Separator(self, orient=HORIZONTAL)
        line.pack(fill='x')

        payment_label = Label(self, text="Select View", bd=10)
        payment_label.pack(anchor='w')

        # Create integer variable
        self.var = IntVar()
        self.var.set(0)  # Use set() initialize the variable
        self.current_view = ["No Filter", "Binary", "Mask"]

        for i, method in enumerate(self.current_view):
            self.curr_v = Radiobutton(
                self, text=method, variable=self.var, value=i)
            self.curr_v.pack(anchor='w')

        # Use ttk to add styling to button
        style = ttk.Style()
        style.configure('TButton', bg='skyblue', fg='white')

        # Create button that will call the method to display text and
        # close the program
        next_button = ttk.Button(self, text="Apply", command=self.printResults)
        next_button.pack()

        # Add canvas for displaying frame
        self.canvas = Canvas(self, width=400, height=300)
        self.canvas.pack()

        # Convert frame to PhotoImage object for display in canvas
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGBA)
        self.frame = cv2.resize(frame, (400, 300))
        self.photo = PhotoImage(image=Image.fromarray(self.frame))
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw')

    def printResults(self):
        """Print the results of the checkboxes and radio buttons."""

        index = self.var.get()
        print("View: {}".format(self.current_view[index]))
        self.quit()
