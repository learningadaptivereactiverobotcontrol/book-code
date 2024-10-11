import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pybullet_zmq",
    version="0.0",
    maintainer="Dominic Reber",
    maintainer_emal="dominic@aica.tech",
    description="This package implements a ZMQ interface for the PyBullet simulation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/epfl-lasa/simulator-backend/tree/develop/pybullet_zmq",
    packages=setuptools.find_packages(),
    install_requires=[
        "control_libraries>=6.0.0",
        "pybullet_simulation",
        "network_interfaces>=1.1.0",
        "pyyaml",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Operating System :: Unix",
    ],
    python_requires='>=3.8',
    scripts=["bin/zmq-simulator"],
)
