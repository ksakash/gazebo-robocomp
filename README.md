# gazebo-robocomp
The following consists of plugins, tools and libraries for the RoboComp framework to communicate with gazbeo.

# Installation

Navigate to your home directory:

`cd`

Now clone the repo:

`git clone https://github.com/ksakash/gazebo-robocomp`

After cloning, move to the project directory:

`cd gazebo-robocomp`

Make a directory to keep all the files together:

`mkdir build`

Move to `build` directory:

`cd build`

Now to build the code:

`cmake ..
make`

All the shared objects (plugins) and some APIs would have been built into the directory.

Now you have to tell gazebo where to find your custom made plugins and models which are loaded at runtime. So, you have to include the file path of both the models and plugins in the gazebo environment variables namely `GAZEBO_MODEL_PATH` and `GAZEBO_PLUGIN_PATH`.

To do it, go to your home directory:

`cd` 

And, add these two lines to your `.bashrc`:

`export GAZEBO_MODEL_PATH = $HOME/gazebo-robocomp/gazebo_robocomp_models/:$GAZEBO_MODEL_PATH`

`export GAZEBO_PLUGIN_PATH = $HOME/gazebo-robocomp/build/:$GAZEBO_PLUGIN_PATH`


Now you have to configure the `.world` files also according to your `username`.

You just have to change `<uri>` element in all the `.world` files and `.sdf` according to your `linux-username`, because it is set to my local `file_path`. Sorry for the inconvenience, I still need to figure out this one.


To test the plugins, just start the gazebo with the respective `.world` file:

`gazebo --verbose laser.world 
gazebo --verbose joint.world` 

You can see the topics being published by:

`gz topic -l`

To see the content of data published:

`gz topic -e <topic_name>`

The laser data will be published on `/gazebo/gazebo_robocomp_laser/hokuyo/hokuyo/link/laser/scan`.

To test the `joint` controller I have made an API for this. You can give desired velocity at which you want to move the joint, when you are in your home directory:

`./joint_vel <desired speed>`

The `joint` plugin will be listening on the topic `~/velodyne/vel_cmd`.

To test the IMU plugin do:

`gazbeo --verbose IMU.world`

The data will start getting published in the respective console.


