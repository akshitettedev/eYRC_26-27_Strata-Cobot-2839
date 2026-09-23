#!/usr/bin/env python3
'''
ore_debug.py — say exactly why an ore was not detected in a saved frame.

Run it on the raw_frame.png your node writes at start-up:

    python3 ore_debug.py raw_frame.png

For each ore type it prints how many pixels the colour threshold caught, then
every blob it found with the numbers the detector judges it on, and whether the
blob was accepted or which gate rejected it.

It also writes one mask image per type, so you can see what the threshold is
actually catching:  mask_azurite_ore.png, mask_malachite_ore.png,
mask_vanadinite_ore.png

It imports the SAME constants your node uses, so the numbers here are the
numbers your node acts on. Run it with ROS sourced, in the same terminal you
would run the node from.
'''

import sys
import os
import cv2
import numpy as np

# Import the live detector so the thresholds cannot drift out of sync.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import ore_detector as od
except ImportError as e:
    print(f'Could not import ore_detector.py: {e}')
    print('Run this from the folder holding ore_detector.py, with ROS sourced:')
    print('    source /opt/ros/jazzy/setup.bash')
    print('    source ~/ros2_ws/install/setup.bash')
    sys.exit(1)


def report_type(hsv, image, ore_type):
    '''
    Description:    Threshold one ore type, then print every blob and its verdict.

    Args:
        hsv         (ndarray):  Frame converted to HSV
        image       (ndarray):  Original BGR frame, for the saved overlay
        ore_type    (str):      One of ore_detector.ore_types

    Returns:
        accepted    (int):      How many blobs passed every gate
    '''

    print(f'\n=== {ore_type} ===')
    accepted_total = 0

    for step, (s_relax, v_relax) in enumerate(od.RELAX_STEPS):
        label = 'strict' if step == 0 else f'relaxed S-{s_relax} V-{v_relax}'
        mask = od.mask_for(hsv, od.hsv_bounds[ore_type], s_relax, v_relax)
        lit = int(np.count_nonzero(mask))
        print(f'\n  pass {step} ({label}): {lit} px passed the colour threshold')

        if lit == 0:
            print('    nothing at all - the HSV bounds are wrong for this ore.')
            print('    Run hsv_tuner.py and click this ore\'s face.')
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        print(f'    {len(contours)} blob(s) found')

        accepted = 0
        for i, c in enumerate(contours):
            info = od.describe_contour(c)
            if info['area'] < 40:
                continue          # pure speckle, not worth a line
            verdict = 'ACCEPTED' if info['ok'] else f"rejected: {info['reason']}"
            print(f"    blob {i}: area={info['area']:7.0f}  "
                  f"aspect={info['rect_aspect']:.2f}  "
                  f"fill={info['rect_fill']:.2f}  -> {verdict}")
            if info['ok']:
                accepted += 1

        print(f'    {accepted} of {od.ORES_PER_TYPE} ores accepted on this pass')
        accepted_total = accepted

        # Save the mask from the pass that was actually used.
        out = f'mask_{ore_type}.png'
        cv2.imwrite(out, mask)

        if accepted >= od.ORES_PER_TYPE:
            break

    return accepted_total


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    image = cv2.imread(sys.argv[1])
    if image is None:
        print(f'Could not read {sys.argv[1]}')
        return

    print(f'Frame: {sys.argv[1]}  ({image.shape[1]} x {image.shape[0]})')
    print(f'Gates: MIN_AREA={od.MIN_AREA}  MAX_AREA={od.MAX_AREA}  '
          f'MAX_RECT_ASPECT={od.MAX_RECT_ASPECT}  '
          f'MIN_RECT_FILL={od.MIN_RECT_FILL}')

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    results = {}
    for ore_type in od.ore_types:
        results[ore_type] = report_type(hsv, image, ore_type)

    print('\n=== summary ===')
    total = 0
    for ore_type, n in results.items():
        total += min(n, od.ORES_PER_TYPE)
        flag = 'ok' if n >= od.ORES_PER_TYPE else 'SHORT'
        print(f'  {ore_type:18s} {n}/{od.ORES_PER_TYPE}  {flag}')
    print(f'  total {total}/6')

    print('\nMasks written: ' +
          ', '.join(f'mask_{t}.png' for t in od.ore_types))
    print('\nHow to read this:')
    print('  0 px passed the threshold   -> HSV bounds are wrong. Run hsv_tuner.py')
    print('  blobs found but all rejected -> a shape gate is too tight; the')
    print('                                  reason names which one and by how much')
    print('  one big blob instead of two  -> the two ores are touching in the mask;')
    print('                                  raise MIN_RECT_FILL or shrink the kernel')


if __name__ == '__main__':
    main()
