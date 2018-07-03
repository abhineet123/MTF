import numpy as np
import os
import cv2, sys

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


def drawRegion(img, corners, color, thickness=1):
    # draw the bounding box specified by the given corners
    for i in range(4):
        p1 = (int(corners[0, i]), int(corners[1, i]))
        p2 = (int(corners[0, (i + 1) % 4]), int(corners[1, (i + 1) % 4]))
        cv2.line(img, p1, p2, color, thickness)
