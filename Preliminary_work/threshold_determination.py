import numpy as np
import matplotlib.pyplot as plt
from movement_detection import *
import os
import cv2
import time


def enumerate_folders(folder):
    folders = [nom for nom in os.listdir(folder)]
    return folders

def enumerate_files(folder):
    files = [os.path.join(folder, file) for file in os.listdir(folder)]
    return files

def detect_movement(image, prev_image):
    image_area = image.shape[0] * image.shape[1]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    prev_gray = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)

    # Calculer la différence avec l'image précédente
    diff = cv2.absdiff(prev_gray, gray)

    # Threshold
    _, threshold = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

    # Opening to reduce the noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)

    # detection of the contours of regions where there is a change
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Filter significant contours
    significant_contours = []
    res_contours = []
    for contour in contours:
            res_contours.append(cv2.contourArea(contour))

    image_with_contours = cv2.drawContours(image.copy(), contours, -1, (0, 255, 0), 2)
    #if __name__ == "__main__":
    #    cv2.imshow('Contours', image_with_contours)
    #    cv2.waitKey(0)

    return np.sum(res_contours)/image_area

without_motion = []
with_motion = []

for folder in enumerate_folders(r"C:\Users\edoua\Documents\Birse\Bristol\MSc Thesis\Mouvement detection"):
     Files = enumerate_files(r"C:\Users\edoua\Documents\Birse\Bristol\MSc Thesis\Mouvement detection" + "\\" + folder)
     without_motion.append(detect_movement(cv2.imread(Files[0]), cv2.imread(str(Files[1])))*100)
     without_motion.append(detect_movement(cv2.imread(Files[2]), cv2.imread(str(Files[3])))*100)
     with_motion.append(detect_movement(cv2.imread(Files[3]), cv2.imread(str(Files[4])))*100)


fig, ax = plt.subplots()
ax.boxplot([without_motion, with_motion], showfliers=False)
ax.axhline(y=0.8, color='red', linestyle='--', label='Ligne à y=50')
ax.text(0.1, 0.08508, 'Moving Threshold', transform=ax.transAxes,
        horizontalalignment='center', color='red')

# Étiquettes des boxplots
etiquettes = ['Static', 'Take-off']
ax.set_xticklabels(etiquettes, fontsize = 15)

# Titre du graphique
plt.ylabel('Proportion of Modified Pixel Surface (%)', fontsize = 15)

# Affichage du graphique
plt.grid()
plt.show()

