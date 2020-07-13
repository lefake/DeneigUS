import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def getDriveways(folder_path):
    imgs = []

    for file in os.listdir(folder_path):
        path = os.path.join(folder_path, file)
        if os.path.isdir(path):
            continue
        imgs.append(cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY))

    return imgs

# Enlarge the image to be able to see the path and not only a image full of gray
def drawPath(img_src, path, path_val=127, scale=3):
    img_path = cv2.resize(img_src, dsize=(len(img_src)*scale, len(img_src[0])*scale))

    for ind, point in enumerate(path):
        x = point[0]
        y = point[1]
        if ind + 1 < len(path):
            direction = (path[ind + 1][0] - path[ind][0], path[ind + 1][1] - path[ind][1])
            for k in range(scale):
                img_path[(x * scale + int((scale-1)/2)) + (direction[0] * k)][(y * scale + int((scale-1)/2)) + (direction[1] * k)] = path_val

    # Highlight start and end of the path
    img_path[path[0][0] * scale + int((scale - 1) / 2)][path[0][1] * scale + int((scale - 1) / 2)] = 200
    img_path[path[-1][0] * scale + int((scale-1)/2)][path[-1][1] * scale + int((scale-1)/2)] = 200

    return img_path

if __name__ == "__main__":
    # Get the maps
    folder_path = "../img"
    driveways = getDriveways(folder_path)
    start_points = [(13, 69), (13, 69), (13, 69), (11, 69), (11, 69), (11, 69)]

    path = [] # Use the algorithm here : List of coordinate in order

    img = drawPath(driveways[-1], path)
    plt.figure()
    plt.imshow(img, cmap="gray")
    
    plt.show()
