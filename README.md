# R26_test

<p align="center">
  <img src="https://github.com/teamrudra/r26_test/blob/main/misc/rover.webp" width="480" height="480"/>

#### Some Instructions
1. You may use any online resources, datasheets, or documentation needed, but be mindful of your time and stay focused on the task.
2. The duration of the test is 90 mins from 5:15pm to 6:45 pm.
3. There will be a MCQ test conducted [here](https://rudra26test.vercel.app/)
4. There are 4 tasks in the tests. Complete all of them.
5. In case you are not able to complete all the tasks, do upload whatever you are able to.
6. In the `README.md` of your repository include your thought process, places where you got stuck, where you used the help of AI, google or other online resources.
7. Even if you are not able to solve anything, do fill the readme and what your thought process would have been.
8. Carefully read the instructions to implement the required functionality.
9. Install [mingw(c compiler)](https://www.mingw-w64.org/downloads/#w64devkit) and [git](https://git-scm.com/downloads) if you haven't already done it.
10. After finishing your test, provide the link to your forked repository in the google form provided at the end.

### Aim/Objective: To decode GPS data of start and goal position, and create a path planning algorithm which computes an optimal path over a predefined gridmap

## Description
You are implementing code to decode GPS position data, received from a u-blox GNSS module on-board a rover (check out the [datasheet](https://drive.google.com/file/d/1rOcPxpP-3JE8l39kBMiQV6KKe8B6zlDf/view)). You are given the current/start position of the rover and the goal position where the rover has to reach, your goal is to develop a path planning algorithm to traverse over a pre-defined gridmap and generate necessary odometry commands (total time & angle traversed) to guide the rover along the generated path. 

### Task 0: Fork the provided repository and ensure it is set to PUBLIC so we can access and assess your work.
### Task 1: Decoding gps data (in ubx format) from u-blox reciever.
Working with UBX format and extracted relevant navigation data for use in the planner.
### Task 2: Develop a path planning algorithm to traverse over a gridmap.
Implemented a grid-based path planner that computes an optimal route from start to goal.
### Task 3: Generate odometry commands to guide the rover along the generated path.
Converted the path into motion commands (direction and timing) based on wheel parameters.
### Task 4: Compile and run the code.
Verified the workflow on sample inputs and ensured the project compiles successfully with g++.

#### Code
1. [src/main.cpp](src/main.cpp): Code for running the test.
2. [src/ublox_reader.cpp](src/ublox_reader.cpp): Recitfy errors in this code to compute correct lat & lon coordinates.
3. [src/planning.cpp](src/planning.cpp): Complete the defined `Planner::pathplanning` function 
4. [src/odometry.cpp](src/odometry.cpp): Complete the defined `Odometry::computeCommands` function 

#### How to Compile & Check your code
(make sure you are in the root directory)   
1. Compile your code by running: `make build`
2. To check if your code is implemented correctly run: `make check`
   
If you are able to compile your code successfully you should see something like this on your screen:

```
*** Success: ***
--------------------------------------------
```

4. If your make check was unsuccesfull, you can clean your attempt by running `make clean`, review your implementation and repeat the previous steps.

# Solution
## steps
to create a public copy of the repository to work on and allow assessment of my work. and debug the initial errors of the code . i forked the github repo to my account and loned it to my local and begin dev.
then Map the environment Represent the terrain as a grid with obstacles and free spaces.
Path planning Compute an optimal route on the grid from start to goal while avoiding obstacles.
Generate motion commands: Convert the planned path into odometry commands distance and heading for the rover.
Compile and execute the program to ensure it correctly reads GPS input, plans a path, and outputs the expected motion commands.
## task 1
## Understanding
read raw UBX data and extracting the start and goal GPS coordinates.
## Thought Process
taking the raw GPS output from a u-blox receiver (which comes in the UBX binary format), reading/decoding it into human-understandable values like latitude, longitude, altitude, speed, and time, and then extracting the pieces of data that are actually useful , comitted all the changes on the way 
## Implementation
To decode the GPS data, I first understood that the UBX format encodes navigation information like latitude, longitude, and height in a binary structure. implemented a parser that reads the raw UBX messages from a file, converts the hexadecimal representation into bytes, and extracts the relevant fields using memcpy. The extracted integer values are then scaled to standard units (degrees for latitude/longitude, meters for height). The readUbloxFile function reads both the start and goal UBX messages and returns them as usable GPS structs for further processing in the planner. This approach ensures precise extraction of navigation data while keeping the implementation modular and easy to extend for additional UBX messages if needed. comitted all changes on the way
## task 2
making it as a 2D grid, marking obstacles wherever they appeared , using a A* algorithm as it effectively find shotest path it seems while avoiding obstacles , next made check all four directions and used a heuristic based on Euclidean distance to guide the search. The output was a list of grid points representing the optimal path from start to goal.
## task 3
this is about how the rover should move ;calculated the total distance by adding up the straight-line distances between consecutive points on the path. I also computed how much the rover would need to turn at each step by checking the change in heading between segments. I summed these angles to get the total turning required wth using a constant speed
Total Distance: Summed straight-line distances between consecutive points.
Heading Changes: Calculated the change in heading between consecutive segments and summed the absolute angles to get total turning required.
Motion Commands: Used a constant speed to calculate time and generated the corresponding angle commands for the rover.
## task 4 
compiled to verify with the test results .

# Google Form
[Link to Repo Submission](https://docs.google.com/forms/d/e/1FAIpQLSdlVJ2LzP8wUOATRD804zDVL611rwwGMO1y_ecYu5aoV5YQfw/viewform)


<p align="center">
  <img src="https://github.com/teamrudra/r25-test/blob/main/datasheets/feynman-simple.jpg" width="600" height="600"/>
</p>
     
