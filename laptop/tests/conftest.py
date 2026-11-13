"""Test setup. Isolates ROS before any test module can initialise rclpy.

The robot setup uses ROS_DOMAIN_ID 19 with SUBNET discovery (see ~/.bashrc), so
tests must never run there: they publish /pirobot/command and would drive a
robot that happens to be on the LAN.
"""
import os

TEST_DOMAIN = os.environ.get("PIROBOT_TEST_ROS_DOMAIN", "88")
if TEST_DOMAIN == "19":
    raise RuntimeError("PIROBOT_TEST_ROS_DOMAIN=19 is the robot's domain; pick another (0-101)")
# Fixed, not random: on the WSL dev laptop some domain IDs silently fail to deliver data.
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.pop("ROS_STATIC_PEERS", None)


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: takes more than a few seconds (ROS integration)")
