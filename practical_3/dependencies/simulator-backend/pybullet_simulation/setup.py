import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pybullet_simulation",
    version="0.0",
    description="This package implements a PyBullet simulation",
    maintainer="Dominic Reber",
    maintainer_emal="dominic@aica.tech",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # url="https://github.com/pypa/sampleproject",
    packages=setuptools.find_packages(),
    install_requires=[
        "control-libraries>=6.0.0",
        "numpy>=1.20.2",
        "numba>=0.54.0",
        "scipy>=1.7.1",
        "pybullet>=3.1.7"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Operating System :: Unix",
    ],
    python_requires='>=3.8',
)
