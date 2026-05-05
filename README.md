# Carla Platooning

###
# Install Carla Package

https://github.com/carla-simulator/carla/releases/tag/0.9.16/

Download this package on your desktop for direct access to your GPU (Not in a VM)
CARLA_0.9.16.tar.gz

Install the package
Run the Carla shell from your desktop
path/to/carla$ ./CarlaUE4.sh

###
# Run Pycarlanet

In Terminal 1

In the VM, run the pycarlanet bridge to connect to Carla using your ip address

for example: 
ubuntu@ubuntu-2204:~/workspace/pycarlanet/joinAtBack$ python3 main.py ip=123.123.123.123 port=2000

It will default to 127.0.0.1, which uses the VM’s loopback, and will not connect to your desktop Carla server. Find your real ip and use that to connect. Carla uses port=2000 by default which is fine. 


###

# Run Omentpp simulation

In Terminal 2

Now open omnetpp and run the simulation

ubuntu@ubuntu-2204:\~/workspace/omnetpp-6.0.3$ source setenv
Environment for 'omnetpp-6.0.3' in directory '/home/ubuntu/workspace/omnetpp-6.0.3' is ready.
ubuntu@ubuntu-2204:~/workspace/omnetpp-6.0.3$ omnetpp

This opens the GUI. Select carla_platooning/examples/joinAtBack/omnetpp.ini and click run (as omnetpp simulation)
Select the full join at back maneuver. 

You should see INIT being sent to the bridge in terminal 1 with a configuration message.
You should also see the carla simulation change maps and being the simulation which lasts 120 seconds. 
