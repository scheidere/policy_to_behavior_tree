from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['mdp_to_bt','rqt_behavior_tree', 'behavior_tree'],
    package_dir={'': 'src'},
)

setup(**d)
