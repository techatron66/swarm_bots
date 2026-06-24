#!/usr/bin/env python3
"""Generate ArUco markers for robot identification."""
import cv2
import os
import argparse

def generate_markers(num_bots=2, size=200, out_dir='./markers'):
    os.makedirs(out_dir, exist_ok=True)
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    except AttributeError:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    for i in range(num_bots):
        try:
            img = cv2.aruco.generateImageMarker(dictionary, i, size)
        except AttributeError:
            img = cv2.aruco.drawMarker(dictionary, i, size)
        path = os.path.join(out_dir, f'marker_{i}.png')
        cv2.imwrite(path, img)
        print(f"Saved {path}")

    # Wall markers
    for wall_id in [10, 11, 12]:
        try:
            img = cv2.aruco.generateImageMarker(dictionary, wall_id, size)
        except AttributeError:
            img = cv2.aruco.drawMarker(dictionary, wall_id, size)
        path = os.path.join(out_dir, f'wall_marker_{wall_id}.png')
        cv2.imwrite(path, img)
        print(f"Saved {path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num', type=int, default=2)
    parser.add_argument('-s', '--size', type=int, default=200)
    parser.add_argument('-o', '--out', default='./markers')
    args = parser.parse_args()
    generate_markers(args.num, args.size, args.out)
