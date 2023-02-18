import tkinter as tk

import cv2 as cv
import numpy as np
from PIL import Image, ImageTk

import pre_process

img_dict = {}

# TODO: pause and continue
# TODO: return without error onClose
# TODO: pass the process into algorithm


class GUI:
    """Creates GUI with tkinter: initializes a master window, creates the same number of radiobuttons as the length of img_dict,
    creates 2 sliders for brightness and saturation, then render images accordingly\n
    :param `master` = tk.Tk()
    :param `img_dict` = { "frameType1": numpy.ndarray }
    :param `window_name` = String
    """

    def __init__(self, master, img_dict, window_name):
        self.master = master
        self.master.title(window_name)

        self.current_avg_brightness = 110
        self.current_avg_saturation = 105
        self.fps = 0

        self.var = tk.IntVar()
        self.curr_selected = 1

        self.frame = tk.Frame(self.master, bg="white", height=600, width=600)
        self.frame.pack(fill="both", expand="yes")

        self.label = tk.Label(self.frame, bg="white", width=450, height=450)
        self.label.pack(fill="both", expand="yes")

        self.fps_label = tk.Label(self.master, text="Frames Per Second: " + str(self.fps))
        self.fps_label.pack(pady=10, side="bottom")

        for i, (img_name, img_array) in enumerate(img_dict.items(), 1):
            radiobutton = tk.Radiobutton(
                self.master, text=img_name, variable=self.var, value=i, command=lambda i=i: self.update_frameType(i))
            radiobutton.pack(pady=10, side="bottom")

        self.brightness_scale = tk.Scale(self.master, from_=0, to=255, orient="horizontal",
                                         label="Brightness", command=lambda x: self.update_brightness(x))
        self.brightness_scale.set(self.current_avg_brightness)
        self.brightness_scale.pack(pady=10, side="bottom")

        self.saturation_scale = tk.Scale(self.master, from_=0, to=255, orient="horizontal",
                                         label="Saturation", command=lambda x: self.update_saturation(x))
        self.saturation_scale.set(self.current_avg_saturation)
        self.saturation_scale.pack(pady=10, side="bottom")


        self.render_image()

    def onClose(self):
        self.master.destroy()

    def update_fps(self, next):
        self.fps = next

    def update_frameType(self, next):
        self.curr_selected = next

    def update_brightness(self, value):
        self.current_avg_brightness = int(value)

    def update_saturation(self, value):
        self.current_avg_saturation = int(value)

    def apply_filter(self, img):
        pre_process.ACCEPTABLE_DIFFERENCE = 0
        pre_process.BRIGHTNESS_BASELINE = self.current_avg_brightness
        pre_process.SATURATION_BASELINE = self.current_avg_saturation
        return pre_process.standardize_frame(img)

    def render_image(self):
        global img_dict
        curr_img = list(img_dict.values())[self.curr_selected - 1]
        if np.any(curr_img != np.zeros((100, 100))):
            curr_img = self.apply_filter(curr_img)

        curr_img = np.array(curr_img).astype(np.uint8)
        curr_img = cv.cvtColor(curr_img, cv.COLOR_BGR2RGB)

        curr_img = Image.fromarray(curr_img)
        curr_img = ImageTk.PhotoImage(curr_img)
        self.label.config(image=curr_img)
        self.label.image = curr_img
        self.fps_label.config(text="Frames Per Second: " + str(self.fps))
        self.master.update()


    def update_dict(self, key_value):
        global img_dict
        img_dict.update(key_value)


def startGUI(window_name, **kwargs):
    """Constructs GUI: pass in a `window name` and at least 1 name of frame: `name1="my_chosen_name_1", name2="my_chosen_name_2", ...`\n
    use `update_dict({"my_chosen_name_1": numpy.ndarray})` to update image\n
    call `render_image()` after updating dict to render\n
    :param `window_name` of type String
    :(param >= 1) `name1="some string"`, `name2="some string"`, `name3="some string"`, ...
    """
    global img_dict
    # for i in range(numberOfViews):
    #     frame_type = "frameType" + str(i + 1)
    #     img_dict[frame_type] = np.zeros((100, 100))
    curr_name = "name"
    curr_index = 1
    if kwargs == {}:
        raise KeyError("kwargs is empty in startGUI(), start with name1= ... ")

    for key, value in kwargs.items():
        if key == curr_name+str(curr_index):
            frame_type = value
            img_dict[frame_type] = np.zeros((100, 100))
            curr_index += 1
        else:
            raise KeyError("argument " + curr_name +
                           str(curr_index) + " is missing in startGUI()")

    root = tk.Tk()
    app = GUI(root, img_dict, window_name)
    return app
