# Leader-follower formation tracking control of quadrotor UAVs using bearing measurements

This repository was used to perform the experiments discussed in "Leader-follower formation tracking control of quadrotor UAVs using bearing measurements". 
The theory in this paper concerns the formation tracking problem of a group of quadrotor UAVs under a limited sensing graph topology.
Agents only have access to bearing measurements, as well as relative velocity with respect to other agents.
Besides that, they have access to their own attitude.
Using persistence of excitation, as little as one neighbor is required for each follower drone to still attain the desired formation.

This repository relies on the [Pegasus GNC project](https://pegasusresearch.github.io/pegasus/index.html).
For the installation of this simulator environment, see [https://pegasusresearch.github.io/pegasus/source/setup/installation.html](https://pegasusresearch.github.io/pegasus/source/setup/installation.html)

If the Pegasus GNC project is installed, the following commands can be executed to perform a simulation:

```
cd ~
mkdir pegasus
cd pegasus
git clone https://github.com/SDoodeman/bpe_controller.git src
colcon build --symlink-install
source ~/pegasus/install/setup.bash
```

Then, in one terminal run

```
ros2 launch bpe sim.launch.py
```

(in case of a real experiment, run `ros2 launch bpe real.launch.py`)

And in another terminal, run

```
ros2 run bpe_missions mission
```
