from setuptools import setup, find_packages

setup(
    name="nav2-pose-sampler",
    version="0.1.0",
    description="Sample random start/goal poses from ROS occupancy grid maps",
    author="Forough Zamani",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "Pillow",
        "scipy",
        "PyYAML",
        "matplotlib",
    ],
    entry_points={
        "console_scripts": [
            "nav2-pose-sampler=main:main",
        ],
    },
)
