# ERIS

This core module allows four phases of memory processing: memory formation, consolidation, revision, and recollection. 

## Instruction

1. Confirm that a folder within your home directory called `memtest` is exist. If not, simply create it. This will be the folder where all the memory data, a.k.a. the long-term memory storage, (currently, Episodic Memory and Semantic Memory) are consolidated.

2. Launch the ERIS core module.

	```
	roslaunch eris core.launch
	```

3. Run `scenectrl` for a manual scene capture in a separate terminal.

	```
	rosrun visor scenectrl
	```

	In the `scenectrl` terminal, press `spacebar` to trigger manually the image processing captured by Baxter, or `c` to cycle the input scene images.

Check the generated memory in the `memtest` folder.

## Launch query servers for Human-Robot Interaction (HRI)

To do so simultaneously with the ERIS core module, uncomment the line below in the `core.launch` of the ERIS pkg:

```
<!--include file="$(find eris)/launch/qservers.xml" /-->
```

To do so separately (this is the default option), do the following:

```
roslaunch eris hri.launch
```

To pose question, when using the `imageloader` module, confirm that a folder called `scene_snap/BaxterVision/` within your home directory is exist, and contains an image file labeled as `vision.JPG` to simulate the visual feed using a camera. Then, run the help command for the `query client` to see more details about the questions and parameters:

```
rosrun eris query_client -h
```

Example question: to pose question 1 of Case 1, execute `rosrun eris query_client -c 1 -q 1`.


## Using the `dmc_node` to read & write memory contents

A `dmc_node` has been implemented to independently read memory contents and create a dummy memory data. 
*Keep in mind that the format of the message must be manually considered, and any detected discrepancies with msg file format will result in an error!*

For more details, check out the help using:

```
rosrun eris dmc_node -h
```


## To move Baxter's arm sequentially:

1. **TURN OFF THE JOINT TRAJECTORY ACTION SERVER** !!! (Causes stutter of movements if turned on)
2. Sort the file in the directory `sequence1`.
	
	To change the directory, adjust parameter in `eris ferdian.py` line 170.

4. Run the movement server.

    ```
    rosrun eris ferdianServer.py
    ```

5. Run the service to start the sequence. 
    
    ```
    rosrun eris ferdian.py
    ```

To suddenly cancel the sequence during movement, press `Ctrl+c`.

Every time Baxter finish moving, press `spacebar` in `scenectrl` terminal to manually capture the scene. (see "Instruction")

## To use ROSTEST:

Put all the `.test` source file in the `test` Folder within the pkg.

Then modify the following code at the bottom of `CMakelists.txt`:

```yaml
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_module test/core.test test/testsource.cpp)
  target_link_libraries(test_module ${catkin_LIBRARIES} dependentNode)  

  add_rostest_gtest(...)
  target_link_libraries(...)
endif()
```

**Note:** test node name must begin with the word `test_` as in >> `test_<nodeName>`, e.g., `test_module`.

To run tests, go to the `build` folder and:

```
make run_tests
```

or to **Run** test per package individually **(NOT COMPILE)**:

```
cd catkin_ws
catkin_make run_tests_packageName
```
