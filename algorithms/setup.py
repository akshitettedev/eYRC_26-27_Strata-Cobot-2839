from setuptools import setup

package_name = 'algorithms'

# Every node you write goes in this list, and then `colcon build` puts it where
# `ros2 run algorithms <name>` can find it.
#
# The executable name is the FILE NAME, .py included:
#
#     ros2 run algorithms task1b.py
#
# `ros2 pkg executables algorithms` lists whatever is currently installed, which is the
# quickest way to check a new file actually made it in. Before you add anything it is
# empty, and that is correct.
#
# START BY COPYING A BOILERPLATE. The three in boilerplate/ are the starting points and
# are deliberately NOT in this list: copy the one you want into the matching
# scripts/task1<x>/ folder under your own name, and add that copy here. The originals
# stay clean to refer back to.
#
#     cp boilerplate/task1b_boilerplate.py scripts/task1b/task1b.py
#     chmod +x scripts/task1b/task1b.py
#
# Three things that catch people out:
#
#   * a file added here is not installed until you `colcon build` again;
#   * `ros2 run` starts the INSTALLED copy, not the one open in your editor, so rebuild
#     and re-source after every edit;
#   * a script needs `#!/usr/bin/env python3` on its first line and the executable bit
#     (`chmod +x`), or it installs fine and then refuses to start.
SCRIPTS = [
    # Your nodes go here, for example:
    'scripts/task1a/ore_detector.py',
    # 'scripts/task1b/task1b.py',
    # 'scripts/task1c/task1c.py',
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e-Yantra',
    maintainer_email='support@e-yantra.org',
    description='Your Task solutions.',
    license='Apache-2.0',
    scripts=SCRIPTS,
)
