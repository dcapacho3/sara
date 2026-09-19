from setuptools import setup
import glob
import os

package_name = 'sara'

# Find all scripts in the 'scripts' and 'debug' folders
scripts = glob.glob('scripts/*.py')
debug_scripts = glob.glob('debug/*.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[os.path.splitext(os.path.basename(script))[0] for script in scripts + debug_scripts],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Capacho',
    maintainer_email='dcapacho3@gmail.com',
    description='SARA (Smart Autonomous Retail Assistant): autonomous shopping-cart robot for structured retail environments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'{os.path.splitext(os.path.basename(script))[0]} = scripts.{os.path.splitext(os.path.basename(script))[0]}:main'
            for script in scripts
        ] + [
            f'{os.path.splitext(os.path.basename(script))[0]} = debug.{os.path.splitext(os.path.basename(script))[0]}:main'
            for script in debug_scripts
        ],
    },
)

