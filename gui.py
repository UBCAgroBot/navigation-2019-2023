import tkinter as tk

import cv2 as cv
import numpy as np
from PIL import Image, ImageTk

import pre_process

img_dict = {}
isActive = False


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
        self.master.geometry("600x700")

        self.current_avg_brightness = 110
        self.current_avg_saturation = 105
        self.fps = 0

        self.var = tk.IntVar()
        self.curr_selected = 1

        self.container = tk.Frame(self.master)
        self.container.pack(side='top', anchor='nw', fill="both", expand=True)

        self.canvas = tk.Canvas(self.container)
        self.scrollbar = tk.Scrollbar(
            self.container, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas)
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(
            scrollregion=self.canvas.bbox("all")))

        self.img_container = tk.Label(self.scrollable_frame, padx=10, pady=10)
        self.img_container.pack(side='top', fill="both", expand=True)
        # self.img_container.config(width=600, height=400)

        self.canvas.create_window(
            (0, 0), window=self.scrollable_frame, anchor='c')
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        # self.canvas.config(width=600, height=300)
        self.scrollbar.pack(side="right", fill="y")

        self.fps_label = tk.Label(
            self.scrollable_frame, text="Frames Per Second: " + str(self.fps))
        self.fps_label.pack(pady=5, side="bottom")

        for i, (img_name, img_array) in enumerate(img_dict.items(), 1):
            radiobutton = tk.Radiobutton(
                self.scrollable_frame, text=img_name, variable=self.var, value=i, command=lambda i=i: self.update_frameType(i))
            radiobutton.pack(pady=5, side="bottom")

        self.brightness_scale = tk.Scale(self.scrollable_frame, from_=0, to=255, orient="horizontal",
                                         label="Brightness", command=lambda x: self.update_brightness(x))
        self.brightness_scale.set(self.current_avg_brightness)
        self.brightness_scale.pack(pady=5, side="bottom")

        self.saturation_scale = tk.Scale(self.scrollable_frame, from_=0, to=255, orient="horizontal",
                                         label="Saturation", command=lambda x: self.update_saturation(x))
        self.saturation_scale.set(self.current_avg_saturation)
        self.saturation_scale.pack(pady=5, side="bottom")
        #
        #
        self.upper_frame = tk.Frame(self.scrollable_frame)
        self.upper_frame.pack(in_=self.scrollable_frame,
                              anchor="c", side="bottom")
        self.lower_frame = tk.Frame(self.scrollable_frame)
        self.lower_frame.pack(in_=self.scrollable_frame,
                              anchor="c", side="bottom")
        self.alert_for_hsv = tk.Label(
            self.lower_frame, text="", fg="red")
        self.alert_for_hsv.pack(pady=5, side="top")
        #
        #
        self.LOWER_GREEN = [35, 80, 80]
        self.low_h_entry_label = tk.Label(self.lower_frame, text="LOWER-H:")
        self.low_h_entry_label.pack(pady=5, side="left")
        self.low_h_entry = tk.Entry(
            self.lower_frame, width=5, justify="center", font="Courier 12")
        self.low_h_entry.insert(0, str(self.LOWER_GREEN[0]))
        self.low_h_entry.pack(pady=5, side="left")
        #
        self.low_s_entry_label = tk.Label(self.lower_frame, text="S:")
        self.low_s_entry_label.pack(pady=5, side="left")
        self.low_s_entry = tk.Entry(
            self.lower_frame, width=5, justify="center", font="Courier 12")
        self.low_s_entry.insert(0, str(self.LOWER_GREEN[1]))
        self.low_s_entry.pack(pady=5, side="left")
        #
        self.low_v_entry_label = tk.Label(self.lower_frame, text="V:")
        self.low_v_entry_label.pack(pady=5, side="left")
        self.low_v_entry = tk.Entry(
            self.lower_frame, width=5, justify="center", font="Courier 12")
        self.low_v_entry.insert(0, str(self.LOWER_GREEN[2]))
        self.low_v_entry.pack(pady=5, side="left")
        #
        self.update_btn_low = tk.Button(
            self.lower_frame, text="Update", command=self.update_lower_hsv)
        self.update_btn_low.pack(pady=0, side="left")
        #
        #
        self.UPPER_GREEN = [80, 255, 255]
        self.up_h_entry_label = tk.Label(self.upper_frame, text="UPPER-H:")
        self.up_h_entry_label.pack(pady=5, side="left")
        self.up_h_entry = tk.Entry(
            self.upper_frame, width=5, justify="center", font="Courier 12")
        self.up_h_entry.insert(0, str(self.UPPER_GREEN[0]))
        self.up_h_entry.pack(pady=5, side="left")
        #
        self.up_s_entry_label = tk.Label(self.upper_frame, text="S:")
        self.up_s_entry_label.pack(pady=5, side="left")
        self.up_s_entry = tk.Entry(
            self.upper_frame, width=5, justify="center", font="Courier 12")
        self.up_s_entry.insert(0, str(self.UPPER_GREEN[1]))
        self.up_s_entry.pack(pady=5, side="left")
        #
        self.up_v_entry_label = tk.Label(self.upper_frame, text="V:")
        self.up_v_entry_label.pack(pady=5, side="left")
        self.up_v_entry = tk.Entry(
            self.upper_frame, width=5, justify="center", font="Courier 12")
        self.up_v_entry.insert(0, str(self.UPPER_GREEN[2]))
        self.up_v_entry.pack(pady=5, side="left")
        #
        self.update_btn_high = tk.Button(
            self.upper_frame, text="Update", command=self.update_upper_hsv)
        self.update_btn_high.pack(pady=0, side="left")
        #
        #
        #
        self.render_image()

    def update_upper_hsv(self):
        upper_h = int(self.up_h_entry.get())
        upper_s = int(self.up_s_entry.get())
        upper_v = int(self.up_v_entry.get())
        if upper_h > self.LOWER_GREEN[0] and upper_s > self.LOWER_GREEN[1] and upper_v > self.LOWER_GREEN[2] and upper_h <= 255 and upper_s <= 255 and upper_v <= 255:
            self.UPPER_GREEN = (upper_h, upper_s, upper_v)
            self.alert_for_hsv.config(
                text="new UPPER-HSV: " + str(self.UPPER_GREEN))
        else:
            warning = "INVALID: "
            if upper_h <= self.LOWER_GREEN[0]:
                warning += "UPPER_H <= LOWER_H\t"
            elif upper_s <= self.LOWER_GREEN[1]:
                warning += "UPPER_S <= LOWER_S\t"
            elif upper_v <= self.LOWER_GREEN[2]:
                warning += "UPPER_V <= LOWER_V\t"
            else:
                warning += "UPPER_V > 255\t"

            self.alert_for_hsv.config(
                text=warning)

            self.up_h_entry.delete(0, tk.END)
            self.up_h_entry.insert(
                0, str(self.UPPER_GREEN[0]))
            self.up_s_entry.delete(0, tk.END)
            self.up_s_entry.insert(
                0, str(self.UPPER_GREEN[1]))
            self.up_v_entry.delete(0, tk.END)
            self.up_v_entry.insert(
                0, str(self.UPPER_GREEN[2]))
        # print(self.UPPER_GREEN)

    def update_lower_hsv(self):
        lower_h = int(self.low_h_entry.get())
        lower_s = int(self.low_s_entry.get())
        lower_v = int(self.low_v_entry.get())
        if lower_h < self.UPPER_GREEN[0] and lower_s < self.UPPER_GREEN[1] and lower_v < self.UPPER_GREEN[2] and lower_h >= 0 and lower_s >= 0 and lower_v >= 0:
            self.LOWER_GREEN = (lower_h, lower_s, lower_v)
            self.alert_for_hsv.config(
                text="new LOWER-HSV: " + str(self.LOWER_GREEN))
        else:
            warning = "INVALID: "
            if lower_h >= self.UPPER_GREEN[0]:
                warning += "UPPER_H <= LOWER_H\t"
            elif lower_s >= self.UPPER_GREEN[1]:
                warning += "UPPER_S <= LOWER_S\t"
            elif lower_v >= self.UPPER_GREEN[2]:
                warning += "UPPER_V <= LOWER_V\t"
            else:
                warning += "LOWER_V < 0\t"

            self.alert_for_hsv.config(
                text=warning)

            self.low_h_entry.delete(0, tk.END)
            self.low_h_entry.insert(
                0, str(self.LOWER_GREEN[0]))
            self.low_s_entry.delete(0, tk.END)
            self.low_s_entry.insert(
                0, str(self.LOWER_GREEN[1]))
            self.low_v_entry.delete(0, tk.END)
            self.low_v_entry.insert(
                0, str(self.LOWER_GREEN[2]))
        # print(self.LOWER_GREEN)

    def getLowerHSV(self):
        return self.LOWER_GREEN

    def getUpperHSV(self):
        return self.UPPER_GREEN

    def update_fps(self, next):
        self.fps = next

    def update_frameType(self, next):
        self.curr_selected = next

    def update_brightness(self, value):
        self.current_avg_brightness = int(value)

    def update_saturation(self, value):
        self.current_avg_saturation = int(value)

    def update_dict(self, key_value):
        global img_dict
        img_dict.update(key_value)

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

        # Get the dimensions of the available space
        max_width = self.canvas.winfo_width()
        max_height = self.canvas.winfo_height()

        # Get the dimensions of the image
        img_width, img_height = curr_img.size

        # Calculate the scale factor to fit the image within the available space
        scale_factor = min(max_width / img_width, max_height / img_height)

        # Resize the image to fit within the available space, while maintaining aspect ratio
        new_width = int(img_width * scale_factor)
        new_height = int(img_height * scale_factor)
        if new_height > 10 and new_width > 10:
            new_width -= 10
            new_height -= 10
        curr_img = curr_img.resize((new_width, new_height), Image.ANTIALIAS)

        curr_img = ImageTk.PhotoImage(curr_img)
        self.img_container.config(image=curr_img, padx=10, pady=10)
        self.img_container.image = curr_img
        self.fps_label.config(
            text="Frames Per Second: ~" + str(self.fps), padx=10, pady=10)
        self.master.update()

    def isActive(self):
        global isActive
        return isActive


global root


def onClose():
    global isActive
    isActive = False
    root.destroy()


def startGUI(window_name, **kwargs):
    """Constructs GUI: pass in a `window name` and at least 1 name of frame: `name1="my_chosen_name_1", name2="my_chosen_name_2", ...`\n
    use `update_dict({"my_chosen_name_1": numpy.ndarray})` to update image\n
    call `render_image()` after updating dict to render\n
    :param `window_name` of type String
    :(param >= 1) `name1="some string"`, `name2="some string"`, `name3="some string"`, ...
    """
    global img_dict, isActive
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
    global root
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClose)
    app = GUI(root, img_dict, window_name)
    isActive = True
    return app