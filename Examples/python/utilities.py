import numpy as np
import os
import cv2, sys


col_rgb = {
    'snow': (250, 250, 255),
    'snow_2': (233, 233, 238),
    'snow_3': (201, 201, 205),
    'snow_4': (137, 137, 139),
    'ghost_white': (255, 248, 248),
    'white_smoke': (245, 245, 245),
    'gainsboro': (220, 220, 220),
    'floral_white': (240, 250, 255),
    'old_lace': (230, 245, 253),
    'linen': (230, 240, 240),
    'antique_white': (215, 235, 250),
    'antique_white_2': (204, 223, 238),
    'antique_white_3': (176, 192, 205),
    'antique_white_4': (120, 131, 139),
    'papaya_whip': (213, 239, 255),
    'blanched_almond': (205, 235, 255),
    'bisque': (196, 228, 255),
    'bisque_2': (183, 213, 238),
    'bisque_3': (158, 183, 205),
    'bisque_4': (107, 125, 139),
    'peach_puff': (185, 218, 255),
    'peach_puff_2': (173, 203, 238),
    'peach_puff_3': (149, 175, 205),
    'peach_puff_4': (101, 119, 139),
    'navajo_white': (173, 222, 255),
    'moccasin': (181, 228, 255),
    'cornsilk': (220, 248, 255),
    'cornsilk_2': (205, 232, 238),
    'cornsilk_3': (177, 200, 205),
    'cornsilk_4': (120, 136, 139),
    'ivory': (240, 255, 255),
    'ivory_2': (224, 238, 238),
    'ivory_3': (193, 205, 205),
    'ivory_4': (131, 139, 139),
    'lemon_chiffon': (205, 250, 255),
    'seashell': (238, 245, 255),
    'seashell_2': (222, 229, 238),
    'seashell_3': (191, 197, 205),
    'seashell_4': (130, 134, 139),
    'honeydew': (240, 255, 240),
    'honeydew_2': (224, 238, 244),
    'honeydew_3': (193, 205, 193),
    'honeydew_4': (131, 139, 131),
    'mint_cream': (250, 255, 245),
    'azure': (255, 255, 240),
    'alice_blue': (255, 248, 240),
    'lavender': (250, 230, 230),
    'lavender_blush': (245, 240, 255),
    'misty_rose': (225, 228, 255),
    'white': (255, 255, 255),
    'black': (0, 0, 0),
    'dark_slate_gray': (79, 79, 49),
    'dim_gray': (105, 105, 105),
    'slate_gray': (144, 138, 112),
    'light_slate_gray': (153, 136, 119),
    'gray': (190, 190, 190),
    'light_gray': (211, 211, 211),
    'midnight_blue': (112, 25, 25),
    'navy': (128, 0, 0),
    'cornflower_blue': (237, 149, 100),
    'dark_slate_blue': (139, 61, 72),
    'slate_blue': (205, 90, 106),
    'medium_slate_blue': (238, 104, 123),
    'light_slate_blue': (255, 112, 132),
    'medium_blue': (205, 0, 0),
    'royal_blue': (225, 105, 65),
    'blue': (255, 0, 0),
    'dodger_blue': (255, 144, 30),
    'deep_sky_blue': (255, 191, 0),
    'sky_blue': (250, 206, 135),
    'light_sky_blue': (250, 206, 135),
    'steel_blue': (180, 130, 70),
    'light_steel_blue': (222, 196, 176),
    'light_blue': (230, 216, 173),
    'powder_blue': (230, 224, 176),
    'pale_turquoise': (238, 238, 175),
    'dark_turquoise': (209, 206, 0),
    'medium_turquoise': (204, 209, 72),
    'turquoise': (208, 224, 64),
    'cyan': (255, 255, 0),
    'light_cyan': (255, 255, 224),
    'cadet_blue': (160, 158, 95),
    'medium_aquamarine': (170, 205, 102),
    'aquamarine': (212, 255, 127),
    'dark_green': (0, 100, 0),
    'dark_olive_green': (47, 107, 85),
    'dark_sea_green': (143, 188, 143),
    'sea_green': (87, 139, 46),
    'medium_sea_green': (113, 179, 60),
    'light_sea_green': (170, 178, 32),
    'pale_green': (152, 251, 152),
    'spring_green': (127, 255, 0),
    'lawn_green': (0, 252, 124),
    'chartreuse': (0, 255, 127),
    'medium_spring_green': (154, 250, 0),
    'green_yellow': (47, 255, 173),
    'lime_green': (50, 205, 50),
    'yellow_green': (50, 205, 154),
    'forest_green': (34, 139, 34),
    'olive_drab': (35, 142, 107),
    'dark_khaki': (107, 183, 189),
    'khaki': (140, 230, 240),
    'pale_goldenrod': (170, 232, 238),
    'light_goldenrod_yellow': (210, 250, 250),
    'light_yellow': (224, 255, 255),
    'yellow': (0, 255, 255),
    'gold': (0, 215, 255),
    'light_goldenrod': (130, 221, 238),
    'goldenrod': (32, 165, 218),
    'dark_goldenrod': (11, 134, 184),
    'rosy_brown': (143, 143, 188),
    'indian_red': (92, 92, 205),
    'saddle_brown': (19, 69, 139),
    'sienna': (45, 82, 160),
    'peru': (63, 133, 205),
    'burlywood': (135, 184, 222),
    'beige': (220, 245, 245),
    'wheat': (179, 222, 245),
    'sandy_brown': (96, 164, 244),
    'tan': (140, 180, 210),
    'chocolate': (30, 105, 210),
    'firebrick': (34, 34, 178),
    'brown': (42, 42, 165),
    'dark_salmon': (122, 150, 233),
    'salmon': (114, 128, 250),
    'light_salmon': (122, 160, 255),
    'orange': (0, 165, 255),
    'dark_orange': (0, 140, 255),
    'coral': (80, 127, 255),
    'light_coral': (128, 128, 240),
    'tomato': (71, 99, 255),
    'orange_red': (0, 69, 255),
    'red': (0, 0, 255),
    'hot_pink': (180, 105, 255),
    'deep_pink': (147, 20, 255),
    'pink': (203, 192, 255),
    'light_pink': (193, 182, 255),
    'pale_violet_red': (147, 112, 219),
    'maroon': (96, 48, 176),
    'medium_violet_red': (133, 21, 199),
    'violet_red': (144, 32, 208),
    'violet': (238, 130, 238),
    'plum': (221, 160, 221),
    'orchid': (214, 112, 218),
    'medium_orchid': (211, 85, 186),
    'dark_orchid': (204, 50, 153),
    'dark_violet': (211, 0, 148),
    'blue_violet': (226, 43, 138),
    'purple': (240, 32, 160),
    'medium_purple': (219, 112, 147),
    'thistle': (216, 191, 216),
    'green': (0, 255, 0),
    'magenta': (255, 0, 255)
}

def readGroundTruth(filename):
    if not os.path.isfile(filename):
        print("Tracking data file not found:\n ", filename)
        sys.exit()

    data_file = open(filename, 'r')
    header = data_file.readline()
    lines = data_file.readlines()
    no_of_lines = len(lines)
    data_array = np.zeros((no_of_lines, 8))
    line_id = 0
    for line in lines:
        words = line.split()
        del words[0]
        if len(words) != 8:
            msg = "Invalid formatting on line %d" % line_id + " in file %s" % filename + ":\n%s" % line
            raise SyntaxError(msg)
        coordinates = []
        for word in words:
            coordinates.append(float(word))
        data_array[line_id, :] = coordinates
        line_id += 1
    data_file.close()
    return data_array


def writeCorners(file_id, corners):
    # write the given corners to the file
    corner_str = ''
    for i in range(4):
        corner_str = corner_str + '{:5.2f}\t{:5.2f}\t'.format(corners[0, i], corners[1, i])
    file_id.write(corner_str + '\n')


def drawRegion(img, corners, color, thickness=1, label=None):
    # draw the bounding box specified by the given corners
    for i in range(4):
        p1 = (int(corners[0, i]), int(corners[1, i]))
        p2 = (int(corners[0, (i + 1) % 4]), int(corners[1, (i + 1) % 4]))
        cv2.line(img, p1, p2, col_rgb[color], thickness)

    if label:
        if cv2.__version__.startswith('3'):
            font_line_type = cv2.LINE_AA
        else:
            font_line_type = cv2.CV_AA
        cv2.putText(img, label, (int(corners[0, 0] - 1),  int(corners[0, 1] - 1)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, col_rgb[color], 1, font_line_type)
