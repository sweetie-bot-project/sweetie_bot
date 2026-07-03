from setuptools import find_packages, setup

setup(
    name="sweetie_bot_behavior_synth",
    version="0.1.0",
    description="Synthetic behavioral test harness: synth scenarios on the real ROS system in sim",
    package_dir={"": "src"},
    packages=find_packages("src"),
    python_requires=">=3.8",
    # rospy/actionlib/msgs come from the sourced ROS env; spaCy from system-site. No hard deps.
    install_requires=[],
)
