-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------


# Wiki | Tutorials | Hello World #

- This tutorial covers creating a [ROS Package](http://wiki.ros.org/Packages) and [Node](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) to interface with the competition.
- If you're new to ROS then follow the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) first.
- The ROS tutorial on [Creating a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) will be especially helpful. 
- Afterwards, it is recommended to complete the [GEAR Interface Tutorial](gear_interface.md) before trying this one.

## 1. Creating a Competition Package ##

- Teams will need at least a ROS package and Node to compete.
- This is where you will put code to start the competition, receive orders, receive sensor data, and control the arms.
- See [this ROS package template](https://github.com/usnistgov/ARIAC/tree/master/ariac_example) for example of a package and node that interacts with ARIAC.
- This tutorial will use it as an example.

### 1.1. Setting up a Catkin Workspace ###

First set up a [Catkin workspace](http://wiki.ros.org/catkin/workspaces) in which to build your package.
A workspace is just a set of folders with a conventional structure.
To make one, create a folder with a subfolder named `src`.

```bash
mkdir -p ~/helloworld_ws/src
cd ~/helloworld_ws/src
```

Now we'll make the workspace ready to build ROS packages

```bash
source /opt/ros/melodic/setup.bash
catkin_init_workspace
```

The workspace is now ready to build packages.

### 1.2. Creating a New Package Using the Template ###

[This ROS package template](https://github.com/usnistgov/ARIAC/tree/master/ariac_example) comes with build system files, a sample configuration, an example C++ node, and an example python node.

This tutorial will limit itself to the C++ example.

To use the template, you need to pick a name for your package.
The name must only have lower case characters and underscores.
This tutorial uses the name `ariac_example`.
Anywhere you see `ariac_example`, replace it with your desired package name.

Now that you have the name, create a folder in the `src` folder of the workspace with it.

```bash
mkdir ~/helloworld_ws/src/ariac_example
cd ~/helloworld_ws/src/ariac_example
```

All the files in your package need to go into this folder.

### 1.3. Writing the package.xml ###

Create a file called `package.xml` in the package folder

```bash
touch ~/helloworld_ws/src/ariac_example/package.xml
```

This is called the [package manifest](http://wiki.ros.org/Manifest).
Its job is to describe what is needed to build and run your package in a machine readable format.
Additionally, it marks the root folder of your package.

Many of the entries in the package manifest are used for packagng and releasing your package.
Since you're probably not going to release your package, many fields can be neglected or filled with placeholders.

```xml
<?xml version="1.0"?>
<package format="2">
  <name>ariac_example</name>
  <version>0.1.0</version>
  <description>An example of an ARIAC competitor's package.</description>
  <maintainer email="zeid.kootbally@nist.gov">Zeid Kootbally</maintainer>

  <license>Apache 2.0</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>nist_gear</depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>trajectory_msgs</depend>

</package>
```

You should change a few things:

* Replace `ariac_example` with your package name
* Change the description
* Change the maintainer name and email to your name and email
  * You can have multiple `<maintainer>` tags
* Change the license
* add any additional dependencies your code will need

### 1.4. Writing the CMakeLists.txt ###

ROS packages use CMake, so next create a CMake build file.

```bash
touch ~/helloworld_ws/src/ariac_example/CMakeLists.txt
```

* This must be in the same folder as your package manifest.
* Copy the content from [the example CMakeLists.txt](https://github.com/usnistgov/ARIAC/blob/master/ariac_example/CMakeLists.txt).
* If you want to create more executables or libraries, you'll need to add them here to tell CMake how to build them.

**Important**, If you have been following the tutorial instructions then comment out the call to `catkin_python_setup()`.
You do not need to comment it out if you copied the whole `ariac_example` folder.

You should change the project name from `ariac_example` to your package name.
For more information, see the [ROS documentation on CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt).

### 1.5. ARIAC Competition Configuration ###

You will need a competition config file to tell ARIAC what sensors you will be using.


Create a directory to store the configuration
Then create a file in the `config` folder called `sample_gear_conf.yaml`

```bash
mkdir -p ~/helloworld_ws/src/ariac_example/config
touch ~/helloworld_ws/src/ariac_example/config/sample_gear_conf.yaml
```

For the example tutorial, copy [the content of this file](https://github.com/usnistgov/ARIAC/blob/master/ariac_example/config/sample_gear_conf.yaml) into it.
This is just an example.
When you write your own config file, you will want more sensors to see all of the bins.
After this tutorial, [see this page](../configuration_spec.md) for more information about writing your config file.

## 2. Creating a C++ Node to Interface with the Competition ##

You will need a ROS node to interface with the competition.
Popular languages for ROS nodes are C++ and Python.
This tutorial will cover creating a C++ ROS node.

Create a folder called `src` in the package and create a file for the C++ code.

```bash
mkdir -p ~/helloworld_ws/src/ariac_example/src/
touch ~/helloworld_ws/src/ariac_example/src/ariac_example_node.cpp
```

Copy [the content of this file](https://github.com/usnistgov/ARIAC/blob/master/ariac_example/src/ariac_example_node.cpp) into it.

### 2.1. C++ Includes ###

The first couple lines are including standard C++ libraries.
These allow using some built in functions and types like `std::count_if` and `std::vector`.

```C++
#include <algorithm>
#include <vector>
```

The next include is for ROS.
This lets you use types like `ros::NodeHandle` and call functions like `ros::init()`.

```C++
#include <ros/ros.h>
```

The last block of include statements is for all the ROS message types used.

```c++
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
```

You must include a file for each ROS message or service type you use in your program.
For example, to use the ROS message [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) you must include the file called `sensor_msgs/LaserScan.h`.
That gives you access to a C++ type `sensor_msgs::LaserScan` which you use in your code.

### 2.2. C++ Main Function ###

The `main()` function is the entry point for your program.
It has the following structure:

```C++
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // ... Creation of subscriptions omitted for brevity

  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
```

First, the call to `ros::init()` sets the node's name.
This name must be unique when the node is run (but don't worry, it can be changed at runtime if it isn't).

Next a `ros::NodeHandle` is created.
This gives access to most of the ROS APIs.

An instance of the example class `MyCompetitionClass` is created and set up.
The largest section is the creation of a bunch of subscribers and callbacks; they will be covered in the next section.

Finally, the competition is started and control is given to ROS using `ros::spin()`.
Inside this function ROS will wait for ROS messages and call callbacks.

### 2.3. C++ Creating Publishers and Subscribers ###

The largest section of the main function is where publishers and subscribers are created.
See [the tutorial about C++ ROS publishers and subscribers](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) for more information.

Lets look at one example.

```c++
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
```

The first argument is the name of the topic, `/ariac/orders`.
The second argument is the queue size, which is how many messages to save if your callback is slow to handle them.
The last two arguments say a function `order_callback()` on an instance of `MyCompetitionClass` stored in `comp_class` should be called every time a new order is received.

### 2.4. Starting the Competition ###

You may have noticed the main function called `start_competition()`.

```c++
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}
```

* This function starts the competition by calling the service `/ariac/start_competition`.
* This is part of the [ARIAC competition interface](../competition_interface_documentation.md), and it must be called when your code is ready to begin a trial.
* This function waits for the service to become available first.
* It's important to do this because the service server is in another process, and may not be ready yet.
* Next, `call()` blocks until a response is received.
* The rest of the code checks if the competition was successfully started.

## 3. Trying the Example ##

Before you can run the example, you need to build it.

```bash
cd ~/helloworld_ws
catkin_make
```

Once that succeeds, open a new terminal.
We'll need to source `setup.bash` in the workspace to make our terminal aware of it.

```bash
source ~/helloworld_ws/devel/setup.bash
```

Now we can launch the competition with our configuration.

```bash
rosrun nist_gear gear.py <your_path_to_package>/config/sample.yaml ~/helloworld_ws/src/ariac_example/config/sample_gear_conf.yaml
```

The first config file `sample.yaml` specifies what trial will be run.
The second config file is the one you created that describes where the sensors should go.

Open a third terminal and source your workspace again.
Then run this command to run your example C++ node.

```bash
rosrun ariac_example ariac_example_node
```

The example prints information about the competition to the console, like sensor data being received.
It also commands the robot to move to the zero position.
Be sure to read through all the example code to make sure you understand it.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
