FINAL PROJECT - PATH PLANNING FOR URBAN DRIVING
- Preyash Parikh
- Ajinkya Parwekar
- Akash Agarwal

Python libraries to be imported :-

        -math
	-sys
	-matplotlib.pyplot
	-time

Important Points :-
	
	- Make sure "vrep.py" ,"vrepTest.py" and "vrepConst.py" are present in the same folder as the execution python file(python_final.py). These files can be found at -  programming\remoteApiBindings\python\python. If its 	   	  sim.py and simConst.py file, please replace vrep with sim wherever necessary in the main file.
	
	- File "remoteApi.dll" is also to be placed in the same folder as above. For windows .dll file is needed, for other OS compatible files needed to be placed in same folder.
	
	- V-REP (VREP_map2) scene is also to be placed in the same folder as above.
	
	- To include turtlebot model in your V-REP, add "Turtlebot2.ttm" in installation folder of V-REP at  "models\robots\mobile".

	- Add "simRemoteApi.start(19999)" at start in child script in V-REP.

 
- Play the VREP environment and then run the code.
- (0,0) is at centre of VREP_map
- There is a video for simulation with dynamic obstacle shown. Explanation about the same is explained with Paper attached.	
	