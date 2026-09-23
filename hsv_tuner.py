#!/usr/bin/env python3
'''
hsv_tuner.py — measure real HSV bounds off a saved camera frame.

Usage:
    python3 hsv_tuner.py raw_frame.png

Controls:
    left click   sample the pixel (and its 5x5 neighbourhood) under the cursor
    t            toggle live mask preview using the current samples
    c            clear all samples
    p            print ready-to-paste bounds for the current samples
    q / ESC      quit (prints bounds on the way out)

Sample the SAME ore face several times: the bright middle, a shadowed corner,
and the edge. The printed bounds cover everything you clicked, plus margin.
'''

import sys
import cv2
import numpy as np

# Margin added around the measured min/max, per channel.
H_MARGIN = 6
S_MARGIN = 45
V_MARGIN = 45

samples = []          # list of (h, s, v) as ints
show_mask = False
image = None
hsv = None


def collect(event, x, y, flags, param):
    '''Mouse callback: sample a 5x5 patch around the click.'''
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    y0, y1 = max(0, y - 2), min(hsv.shape[0], y + 3)
    x0, x1 = max(0, x - 2), min(hsv.shape[1], x + 3)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
    for px in patch:
        samples.append((int(px[0]), int(px[1]), int(px[2])))
    print(f'  sampled ({x},{y}) -> H={hsv[y, x][0]} S={hsv[y, x][1]} V={hsv[y, x][2]}'
          f'   [{len(samples)} px total]')


def bounds():
    '''
    Turn the samples into HSV bounds.

    Returns a list of (lower, upper) pairs. Two pairs are returned when the
    hues straddle the 0/179 seam, which is what orange and red do.
    '''
    if not samples:
        return []

    arr = np.array(samples)
    h, s, v = arr[:, 0], arr[:, 1], arr[:, 2]

    s_lo = max(0, int(s.min()) - S_MARGIN)
    s_hi = min(255, int(s.max()) + S_MARGIN)
    v_lo = max(0, int(v.min()) - V_MARGIN)
    v_hi = min(255, int(v.max()) + V_MARGIN)

    # Detect a wrap: hues clustered at both ends of the circle.
    low_end = h[h <= 30]
    high_end = h[h >= 150]
    wrapped = len(low_end) > 0 and len(high_end) > 0

    if wrapped:
        hi_of_low = min(179, int(low_end.max()) + H_MARGIN)
        lo_of_high = max(0, int(high_end.min()) - H_MARGIN)
        return [
            (np.array([0, s_lo, v_lo]), np.array([hi_of_low, s_hi, v_hi])),
            (np.array([lo_of_high, s_lo, v_lo]), np.array([179, s_hi, v_hi])),
        ]

    h_lo = max(0, int(h.min()) - H_MARGIN)
    h_hi = min(179, int(h.max()) + H_MARGIN)
    return [(np.array([h_lo, s_lo, v_lo]), np.array([h_hi, s_hi, v_hi]))]


def report():
    '''Print the bounds in the exact shape the detector expects.'''
    pairs = bounds()
    if not pairs:
        print('\nNo samples taken.')
        return
    print('\n--- paste into hsv_bounds ---')
    parts = []
    for lo, hi in pairs:
        parts.append(f'(np.array([{lo[0]:3d}, {lo[1]:3d}, {lo[2]:3d}]), '
                     f'np.array([{hi[0]:3d}, {hi[1]:3d}, {hi[2]:3d}]))')
    print("    '<ore_type>': [" + ',\n                   '.join(parts) + '],')
    if len(pairs) == 2:
        print('  (two ranges: your hues wrap the 0/179 seam)')
    print('-----------------------------\n')


def main():
    global image, hsv, show_mask

    if len(sys.argv) < 2:
        print(__doc__)
        return

    image = cv2.imread(sys.argv[1])
    if image is None:
        print(f'Could not read {sys.argv[1]}')
        return

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.namedWindow('tuner')
    cv2.setMouseCallback('tuner', collect)
    print(__doc__)

    while True:
        if show_mask and samples:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in bounds():
                mask |= cv2.inRange(hsv, lo, hi)
            view = cv2.bitwise_and(image, image, mask=mask)
        else:
            view = image.copy()

        cv2.imshow('tuner', view)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key == ord('t'):
            show_mask = not show_mask
            print(f'  mask preview {"on" if show_mask else "off"}')
        elif key == ord('c'):
            samples.clear()
            print('  samples cleared')
        elif key == ord('p'):
            report()

    report()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
