# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['benchmark_runner', 'benchmark_runner.task_runner',
              'benchmark_runner.planner_interface', 'benchmark_runner.logging'],
    package_dir={'': 'src'},
)

setup(**setup_args)
