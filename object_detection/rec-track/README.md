ecognition (and tracking)
=============================

An object recognition (and tracking) framework.


Prerequisites
-------------

* TrackerQt 1.1


Installation of TrackerQt
-------------------------

1. Get the source code:
    
        http://users.acin.tuwien.ac.at/tmoerwald/
under download

 
2. Configure and compile:

Follow the README file

        cd TrackerQt && mkdir build && cd build
        ccmake ..
        
    In the interactive menu of ccmake hit `c` for configure. Finally, hit `g` for generate and exit.

    Back on the command line type:

        make
        
    Install into the system (to /usr/local)
    
        sudo make install 


Installation of this package 
-----------------------------

1. Compile the code using catkin

        cd rec-trac
        catkin_make --force-cmake
        

Getting Started
---------------

Get a models archive and unzip somewhere on your disk. 

Start some terminals and run the commands below:

1. Fire up roscore:
   
        $ roscore

2. Start the recognition service (The duration of the training phase depends on how many models you use):

        $ rosrun shape_simple_classifier shape_simple_classifier_node -models_dir /path/to/models/data/ -training_dir /path/to/models/trained/ -nn 10

   Use `-chop_z` to cut off all information beyond a given distance. FOr example: `-chop_z 2`

3. Plug-in the kinect and start openni:

        $ roslaunch openni_launch openni.launch

4. Run a client to test the service:

        $ rosrun soc_test soc_test_node

