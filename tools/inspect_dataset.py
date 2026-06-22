#!/usr/bin/env python3
"""Overlay gate labels (clipped polygon + center + distance/yaw) on every
dataset image, so you can eyeball the annotation quality.

Usage:
    python3 inspect_dataset.py [DATASET_DIR]

DATASET_DIR defaults to ~/pencilnet_dataset. Annotated copies are written to
<DATASET_DIR>/inspect/ with the same filenames.
"""

import json
import os
import sys

import cv2
import numpy as np

DATASET_DIR = os.path.expanduser(
    sys.argv[1] if len(sys.argv) > 1 else '~/pencilnet_dataset')
OUT_DIR = os.path.join(DATASET_DIR, 'inspect')

with open(os.path.join(DATASET_DIR, 'annotations.json')) as f:
    data = json.load(f)

annotations = data['annotations']
os.makedirs(OUT_DIR, exist_ok=True)

colors = [
    (0, 255, 0), (0, 255, 255), (255, 0, 0),
    (0, 0, 255), (255, 0, 255),
]

count = 0
for entry in annotations:
    img_path = os.path.join(DATASET_DIR, 'images', entry['image'])
    img = cv2.imread(img_path)
    if img is None:
        continue

    for j, annot in enumerate(entry['annotations']):
        color = colors[j % len(colors)]

        # Draw polygon from corner points (variable count after clipping)
        if 'corners_px' in annot:
            pts = annot['corners_px']
            n_pts = len(pts)
            for k in range(n_pts):
                p1 = (int(pts[k][0]), int(pts[k][1]))
                p2 = (int(pts[(k + 1) % n_pts][0]), int(pts[(k + 1) % n_pts][1]))
                cv2.line(img, p1, p2, color, 2)
                cv2.circle(img, p1, 4, color, -1)

        # Draw midpoint
        cx, cy = int(annot['center_x']), int(annot['center_y'])
        cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        tag = ' [presence-only]' if annot.get('presence_only') else ''
        label = (f"d={annot['distance']:.1f}m "
                 f"yaw={np.degrees(annot['yaw_relative']):.0f}{tag}")
        cv2.putText(img, label, (cx + 12, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    out_path = os.path.join(OUT_DIR, entry['image'])
    cv2.imwrite(out_path, img)
    count += 1
    if count % 500 == 0:
        print(f"  {count}/{len(annotations)}...")

print(f"Done. Saved {count} images to {OUT_DIR}/")
