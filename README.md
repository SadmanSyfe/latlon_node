# ROS 2 Location Publisher-Subscriber

This project is a ROS 2 setup where a **publisher node** receives location data via a TCP socket server and publishes it as a `geometry_msgs/Vector3` message. A **subscriber node** then listens to this topic and prints the received latitude and longitude.

## Table of Contents

* [Overview](#overview)
* [Getting Started](#getting-started)
    * [Prerequisites](#prerequisites)
    * [Setup](#setup)
* [Usage](#usage)
    * [Running the Publisher Node](#running-the-publisher-node)
    * [Running the Subscriber Node](#running-the-subscriber-node)
* [Node Details](#node-details)
    * [`latlon_publisher.py`](#latlon_publisherpy)
    * [`latlon_subscriber.py`](#latlon_subscriberpy)
* [Topic Information](#topic-information)

## Overview

The `latlon_publisher.py` node acts as a bridge between an external data source (via a TCP socket) and the ROS 2 ecosystem. It listens for incoming JSON messages containing latitude and longitude. Once received, it converts this data into a `geometry_msgs/Vector3` message (using `x` for latitude and `y` for longitude) and publishes it on the `/location_data` topic.

The `latlon_subscriber.py` node simply subscribes to the `/location_data` topic and logs the received latitude and longitude values to the console.

## Getting Started

### Prerequisites

Before you begin, ensure you have:

* **ROS 2 installed:** This project is built for ROS 2. If you don't have it, follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for your operating system.
* **Python 3:** ROS 2 relies on Python 3.
* **`rclpy` and `geometry_msgs`:** These are standard ROS 2 Python libraries and message types, so they should be available with a typical ROS 2 installation.

### Setup

1.  **Create a ROS 2 Workspace (if you don't have one):**

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Create a new ROS 2 package:**

    ```bash
    ros2 pkg create --build-type ament_python your_package_name --dependencies rclpy geometry_msgs
    ```

    Replace `your_package_name` with a descriptive name for your package, e.g., `location_monitor`.

3.  **Place the Python files:**
    Navigate into your package directory (e.g., `~/ros2_ws/src/location_monitor/`). Create a folder named `your_package_name` (e.g., `location_monitor`) inside the `src` subfolder (e.g., `~/ros2_ws/src/location_monitor/location_monitor`). Place `latlon_publisher.py` and `latlon_subscriber.py` inside this second `your_package_name` folder.

    Your directory structure should look something like this:

    ```
    ros2_ws/
    ├── src/
    │   └── your_package_name/
    │       ├── your_package_name/  <-- Place your Python files here
    │       │   ├── latlon_publisher.py
    │       │   └── latlon_subscriber.py
    │       ├── package.xml
    │       └── setup.py
    ```

4.  **Modify `setup.py`:**
    Open `setup.py` in your package directory (`~/ros2_ws/src/your_package_name/setup.py`). You need to add the entry points for your executables. Modify the `entry_points` section to include your publisher and subscriber:

    ```python
    from setuptools import setup

    package_name = 'your_package_name' # Make sure this matches your package name

    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/resource', ['resource/' + package_name]),
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Your Name', # Replace with your name
        maintainer_email='your.email@example.com', # Replace with your email
        description='ROS 2 package for publishing and subscribing to location data.',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'latlon_publisher = your_package_name.latlon_publisher:main',
                'latlon_subscriber = your_package_name.latlon_subscriber:main',
            ],
        },
    )
    ```

    **Remember to replace `your_package_name` with the actual name of your package.**

5.  **Build your package:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select your_package_name
    ```

    Replace `your_package_name` with the actual name of your package.

6.  **Source your workspace:**

    ```bash
    source install/setup.bash
    ```

    You'll need to run this command in every new terminal you open before running your ROS 2 nodes.

## Usage

### Running the Publisher Node

The publisher node (`latlon_publisher`) creates a TCP socket server on `0.0.0.0:8080`. It waits for a client connection and then continuously receives data, parses it as JSON, and publishes it.

Open a new terminal and run:

```bash
ros2 run your_package_name latlon_publisher
```
Replace `your_package_name` with the actual name of your package.

**You should see messages like**:

`[INFO] [latlon_publisher]: Waiting for client connection...`

## Running the Subscriber Node

The subscriber node will listen for messages on the `/location_data` topic and print them.

Open another new terminal and run:
```bash
ros2 run your_package_name latlon_subscriber
```
Replace `your_package_name` with the actual name of your package.

You should see output similar to this as data is sent to the publisher (e.g., from a connected Flutter app or another client sending JSON):
```bash
[INFO] [latlon_subscriber]: Received Location: Latitude=34.0522, Longitude=-118.2437
[INFO] [latlon_subscriber]: Received Location: Latitude=40.7128, Longitude=-74.0060
[INFO] [latlon_subscriber]: Received Location: Latitude=51.5074, Longitude=0.1278
```
## Node Details

### `latlon_publisher.py`

* **Node Name:** `latlon_publisher`
* **Purpose:** Receives JSON-formatted latitude/longitude data over a TCP socket and publishes it as a ROS 2 `geometry_msgs/Vector3` message.
* **Socket Server:**
    * **Address:** `0.0.0.0`
    * **Port:** `8080`
    * Listens for a single client connection.
* **Published Topic:**
    * **Name:** `/location_data`
    * **Type:** `geometry_msgs/Vector3`
    * **Message Mapping:**
        * `Vector3.x` maps to `latitude`
        * `Vector3.y` maps to `longitude`
        * `Vector3.z` is set to `0.0` (unused in this context but required by the message type)

### `latlon_subscriber.py`

* **Node Name:** `latlon_subscriber`
* **Purpose:** Subscribes to the `/location_data` topic and prints the latitude and longitude received from the `latlon_publisher`.
* **Subscribed Topic:**
    * **Name:** `/location_data`
    * **Type:** `geometry_msgs/Vector3`

## Topic Information

You can inspect the `/location_data` topic using ROS 2 command-line tools:

* **List topics:**

    ```bash
    ros2 topic list
    ```
You should see `/location_data` in the list.

* **View topic type:**
    ```bash
    ros2 topic info /location_data
    ```
This will show `Type: geometry_msgs/msg/Vector3`.

* **Echo topic messages (for debugging):**

    ```bash
    ros2 topic echo /location_data
    ```

    This will print the raw `geometry_msgs/Vector3` messages.
