from skbuild import setup

setup(
    name="PyRGBDPixelTo3D",
    version="0.9",
    description="3D Pose for RGB-D cameras and RGBD 2D to 3D converter",
    author='VH',
    license="MIT",
    packages=['PyRGBDPixelTo3D'],
    package_dir={'': 'wrappers'},
    cmake_install_dir='wrappers/PyRGBDPixelTo3D',
    python_requires='>=3.7',
    cmake_args=['-DBUILD_PYTHON_BINDINGS:BOOL=TRUE'],
    include_package_data=True,
    package_data = {'PyRGBDPixelTo3D': ['pose.py']}
)