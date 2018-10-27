import sys, time, collections;
import FOON_classes as FOON;
import ipdb
import re

# -- Last updated: 6/23/2017

FOON_Lvl1 = []; FOON_Lvl2 = []; FOON_Lvl3 = [];

nodes_Lvl1 = []; nodes_Lvl2 = []; nodes_Lvl3 = [];

global totalNodes;
totalNodes = 0;

def FUExists(U, A):
	if A == 1:
		if not FOON_Lvl1:
			return False;
		for F in FOON_Lvl1:
			if F.equals_Lvl1(U):
				return True;
		return False;
	if A == 2:
		if not FOON_Lvl2:
			return False;
		for F in FOON_Lvl2:
			if F.equals_Lvl2(U):
				return True;
		return False;
	elif A == 3:
		if not FOON_Lvl3:
			return False;
		for F in FOON_Lvl3:
			if F.equals_Lvl3(U):
				return True;
		return False;
	else:
		pass;

def constructFUGraph(file_name):
	global totalNodes;
	count = totalNodes; 	# -- 'totalNodes' gives an indication of the number of object AND motion nodes are in FOON.
	stateParts = []; objectParts = []; motionParts = []; # -- objects used to contain the split strings
	objectIndex = -1; # --  variables to hold position of object/motion within list of Things
	isInput = True;

	newFU_lvl3 = FOON.FunctionalUnit(); newFU_lvl2 = FOON.FunctionalUnit(); newFU_lvl1 = FOON.FunctionalUnit(); # object which will hold the functional unit being read.
	_file = open(file_name, 'r');
	items = _file.read().splitlines();

	x = 0;
	while x < len(items):
		line = items[x];
		objectExisting = -1;
		if line.startswith("//"):
			# -- we are adding a new FU, so start from scratch..
			if FUExists(newFU_lvl3, 3) == False:
				nodes_Lvl3.append(newFU_lvl3.getMotion());
				FOON_Lvl3.append(newFU_lvl3);
				count += 1; # -- we only keep track of the total number of nodes in the LVL3 FOON.
			if FUExists(newFU_lvl2, 2) == False:
				nodes_Lvl2.append(newFU_lvl2.getMotion()); # -- no matter what, we add new motion nodes; we will have multiple instances everywhere.
				FOON_Lvl2.append(newFU_lvl2);
			if FUExists(newFU_lvl1, 1) == False:
				nodes_Lvl1.append(newFU_lvl1.getMotion());
				FOON_Lvl1.append(newFU_lvl1);
			newFU_lvl3 = FOON.FunctionalUnit(); newFU_lvl2 = FOON.FunctionalUnit(); newFU_lvl1 = FOON.FunctionalUnit(); # -- create an entirely new FU object to proceed with reading new units.
			isInput = True; # -- this is the end of a FU so we will now be adding input nodes; set flag to TRUE.
		elif line.startswith("O"):
			# -- this is an Object node, so we probably should read the next line one time
			objectParts = line.split("O"); # -- get the Object identifier by splitting first instance of O
			objectParts = objectParts[1].split("\t");

			# -- read the next line containing the Object state information
			x += 1;
			line = items[x];
			stateParts = line.split("S"); # get the Object's state identifier by splitting first instance of S
			stateParts = stateParts[1].split("\t");

		# Functional Unit - Level 3:
			newObject = FOON.Object(int(objectParts[0]), int(stateParts[0]), objectParts[1], stateParts[1]);

			# -- check if this object is a container:
			if len(stateParts) > 2:
				ingredients = [ stateParts[2] ];
				ingredients = ingredients[0].split("{");
				ingredients = ingredients[1].split("}");
				# -- we then need to make sure that there are ingredients to be read.
				if len(ingredients) > 0:
					ingredients = ingredients[0].split(",");
					for I in ingredients:
						newObject.addIngredient(I);

			# -- check if Object node exists in the list of objects
			for N in nodes_Lvl3:
				if isinstance(N, FOON.Object) and N.equals_Lvl3(newObject):
					objectExisting = nodes_Lvl3.index(N);

			# -- check if object already exists within the list so as to avoid duplicates
			if objectExisting != -1:
				objectIndex = objectExisting;
			else:
				# -- just add new object to the list of all nodes
				nodes_Lvl3.append(newObject);
				objectIndex = count;
				count += 1;
			print objectParts
			if isInput == True:
				# -- this Object will be an input node to the FU:
				newFU_lvl3.addObjectNode(nodes_Lvl3[objectIndex], 1, int(objectParts[2]));
			else:
				# -- add the Objects as output nodes to the FU:
				newFU_lvl3.addObjectNode(nodes_Lvl3[objectIndex], 2, int(objectParts[2]));
				newFU_lvl3.getMotion().addNeighbour(newObject); # -- make the connection from Motion to Object

		# Functional Unit - Level 2:
			objectExisting = -1;
			newObject = FOON.Object(int(objectParts[0]), int(stateParts[0]), objectParts[1], stateParts[1]);

			# -- check if Object node exists in the list of objects
			for N in nodes_Lvl2:
				if isinstance(N, FOON.Object) and N.equals_Lvl2(newObject):
					objectExisting = nodes_Lvl2.index(N);

			# -- check if object already exists within the list so as to avoid duplicates
			if objectExisting != -1:
				objectIndex = objectExisting;
			else:
				# -- just add new object to the list of all nodes
				objectIndex = len(nodes_Lvl2);
				nodes_Lvl2.append(newObject);

			if isInput == True:
				newFU_lvl2.addObjectNode(nodes_Lvl2[objectIndex], 1, int(objectParts[2]));
			else:
				newFU_lvl2.addObjectNode(nodes_Lvl2[objectIndex], 2, int(objectParts[2]));
				newFU_lvl2.getMotion().addNeighbour(newObject);

		# Functional Unit - Level 1:
			objectExisting = -1;
			noState = FOON.Object(int(objectParts[0]), objectParts[1]);

			# -- check if Object node exists in the list of objects
			for N in nodes_Lvl1:
				if isinstance(N, FOON.Object) and N.equals_Lvl1(noState):
					objectExisting = nodes_Lvl1.index(N);

			# -- check if object already exists within the list so as to avoid duplicates
			if objectExisting != -1:
				objectIndex = objectExisting;
			else:
				objectIndex = len(nodes_Lvl1);
				nodes_Lvl1.append(noState);

			if isInput == True:
				newFU_lvl1.addObjectNode(nodes_Lvl1[objectIndex], 1, int(objectParts[2]));
			else:
				newFU_lvl1.addObjectNode(nodes_Lvl1[objectIndex], 2, int(objectParts[2]));
				newFU_lvl1.getMotion().addNeighbour(noState);

		else:
			# -- We are adding a Motion node, so very easy to deal with, as follows:
			isInput = False;
			line = items[x];
			motionParts = line.split("M"); # -- get the Motion number...
			motionParts = motionParts[1].split("\t"); #  ... and get the Motion label

		# Functional Unit - Level 3:
			# -- create new Motion based on what was read:
			newMotion = FOON.Motion(int(motionParts[0]), motionParts[1]);
			for T in newFU_lvl3.getInputList():
				T.addNeighbour(newMotion); # -- make the connection from Object(s) to Motion
			newFU_lvl3.setMotion(newMotion);
			newFU_lvl3.setTimes(motionParts[2], motionParts[3]);

		# Functional Unit - Level 2:
			newMotion = FOON.Motion(int(motionParts[0]), motionParts[1]);
			for T in newFU_lvl2.getInputList():
				T.addNeighbour(newMotion);
			newFU_lvl2.setMotion(newMotion);
			newFU_lvl2.setTimes(motionParts[2], motionParts[3]);

		# Functional Unit - Level 1:
			newMotion = FOON.Motion(int(motionParts[0]), motionParts[1]);
			for T in newFU_lvl1.getInputList():
				T.addNeighbour(newMotion);
			newFU_lvl1.setMotion(newMotion);
			newFU_lvl1.setTimes(motionParts[2], motionParts[3]);
		#endif
		x += 1;
	#endwhile
	_file.close(); # -- Don't forget to close the file once we are done!
	return count;
#enddef

def identifyKitchenItems(file_name):
	_file = open(file_name, 'r');
	items = _file.read().splitlines();
	kitchen = [];
	x = 0;
	while x < len(items):
		line = items[x];
		# -- get the Object identifier by splitting first instance of O - indicates the object being used:
		objectParts = line.split("O");
		print objectParts;
		objectParts = objectParts[1].split("\t");

		# -- read the next line containing the object's state information:
		x += 1;
		line = items[x];
		stateParts = line.split("S"); # get the Object's state identifier by splitting first instance of S
		print objectParts
		stateParts = stateParts[1].split("\t");

		# -- create a new object which is equal to the kitchenItem and add it to the list:
		kitchenItem = FOON.Object(int(objectParts[0]), int(stateParts[0]), objectParts[1], stateParts[1]);

		# -- check if this object is a container:
		if len(stateParts) > 2:
			ingredients = [ stateParts[2] ];
			ingredients = ingredients[0].split("{");
			ingredients = ingredients[1].split("}");
			# -- we then need to make sure that there are ingredients to be read.
			if len(ingredients) > 0:
				ingredients = ingredients[0].split(",");
				for I in ingredients:
					kitchenItem.addIngredient(I);
				#endfor
			#endif
		#endif
		kitchen.append(kitchenItem);
		x += 1;
	#endfor
	_file.close();
	return kitchen;

def findAllPaths():
	goalType = raw_input("Please type the goal node's TYPE here: > ");
	goalState = raw_input("Please type the goal node's STATE here: > ");
	goalNode = FOON.Object(int(goalType), int(goalState));

	hierarchy_level = int(raw_input("At what level is the search being done? [1/2/3] > "));

	searchNodes = None; searchFOON = None;

	if hierarchy_level == 1:
		searchNodes = nodes_Lvl1;
		searchFOON = FOON_Lvl1;
	elif hierarchy_level == 2:
		searchNodes = nodes_Lvl2;
		searchFOON = FOON_Lvl2;
	elif hierarchy_level == 3:
		searchNodes = nodes_Lvl3;
		searchFOON = FOON_Lvl3;
	else:
		exit();

	# -- first part: check if the object actually exists in the network:
	index = -1;
	for T in searchNodes:
		if isinstance(T, FOON.Object) and T.equals_functions[hierarchy_level-1](goalNode):
			index = searchNodes.index(T);

	if index == -1:
		print "Item O" + goalNode.getObjectType() + "_S" + goalNode.getStateType() + " has not been found in network!";
		return False;
	#endif

	# What structures do we need in record keeping?
	#	-- a FIFO list of all nodes we need to search (a queue)
	#	-- a list that keeps track of what we have seen
	#	-- a list of all items we have/know how to make in the present case (i.e. the kitchen list)
	itemsToSearch = collections.deque();
	itemsSeen = [];
	kitchen = [];

	goalNode = searchNodes[index];	# this is the actual goal node which is in the network.

	# -- Add the object we wish to search for to the two lists created above:
	itemsToSearch.append(searchNodes[index]);

	FUtoSearch = []; # -- structure to keep track of all candidate functional units in FOON
	tree_allPaths = []; # -- list of ALL possible functional units that can be used for making the item.

	max_iterations = 0; prior = -1; further = -1;
	depth = 100000; # -- maximum number of times you can "see" the original goal node.

	while itemsToSearch:
		# -- Remove the item we are trying to make from the queue of items we need to learn how to make
		tempObject = itemsToSearch.popleft();

		# -- sort-of a time-out for the search if it does not produce an answer dictated by the amount of time it takes.
		if prior == len(itemsToSearch) and further == prior:
			max_iterations += 1;

		if max_iterations > depth:	# just the worst possible way of doing this, but will do for now.
			break;

		further = prior;
		prior = len(itemsToSearch);

		found = False;
		for S in itemsSeen:
			if S.equals_functions[hierarchy_level-1](tempObject):
				found = True;
				break;

		if found == True:
			# just proceed to next iteration, as we already know how to make current item!
			continue;

		for FU in searchFOON:
			# -- searching for all functional units with our goal as output
			found = -1;
			for N in FU.getOutputList():
				if N.equals_functions[hierarchy_level-1](tempObject):
					found += 1;
			# -- only add a functional unit if it produces a specific output.
			if found > -1:
				tree_allPaths.append(FU);
				for N in FU.getInputList():
					itemsToSearch.append(N);
		#endfor
		found = False;
		for N in itemsSeen:
			if N.equals_functions[hierarchy_level-1](tempObject):
				found = True;
		if found == False:
			itemsSeen.append(tempObject);

	# -- saving task tree sequence to file..
	_file = open("FOON_all-paths-to-O" + goalType + "_S" + goalState + "_Lvl" + str(hierarchy_level) + ".txt", 'w');

	for FU in tree_allPaths:
		# -- just write all functional units that were put into the list:
		FU.printFunctionalUnit();
		_file.write(FU.getInputsForFile());
		_file.write(FU.getMotionForFile());
		_file.write(FU.getOutputsForFile());
		_file.write("//\n");
	#endfor
	return True;
#endef

def taskTreeRetrieval():
	environment = identifyKitchenItems('kitting_experiment_objects.txt'); # -- kitchen items needed to find the solution
	# goalType = raw_input("Please type the goal node's TYPE here: > ");
	# goalState = raw_input("Please type the goal node's STATE here: > ");
	goalType = '1'
	goalState = '3'
	goalNode = FOON.Object(int(goalType), int(goalState));

	# hierarchy_level = int(raw_input("At what level is the search being done? [1/2/3] > "));
	hierarchy_level = 2
	searchNodes = None; searchFOON = None;

	if hierarchy_level == 1:
		searchNodes = nodes_Lvl1;
		searchFOON = FOON_Lvl1;
	elif hierarchy_level == 2:
		searchNodes = nodes_Lvl2;
		searchFOON = FOON_Lvl2;
	elif hierarchy_level == 3:
		searchNodes = nodes_Lvl3;
		searchFOON = FOON_Lvl3;
	else:
		exit();

	# -- first part: check if the goal object actually exists in the network (recipe_tea.txt):
	index = -1;
	for T in searchNodes:
		if isinstance(T, FOON.Object) and T.equals_functions[hierarchy_level-1](goalNode):
			index = searchNodes.index(T);

	if index == -1:
		print "Item O" + goalNode.getObjectType() + "_S" + goalNode.getStateType() + " has not been found in network!";
		return False;
	#endif

	# What structures do we need in record keeping?
	#	-- a FIFO list of all nodes we need to search (a queue)
	#	-- a list that keeps track of what we have seen
	#	-- a list of all items we have/know how to make in the present case (i.e. the kitchen list)
	itemsToSearch = collections.deque();
	itemsSeen = [];
	kitchen = [];

	goalNode = searchNodes[index];	# this is the actual goal node which is in the network.

	# -- Add the object we wish to search for to the two lists created above:
	itemsToSearch.append(searchNodes[index]);
	itemsSeen.append(searchNodes[index]);

	FUtoSearch = []; # -- structure to keep track of all candidate functional units in FOON
	taskTree = [];	# -- tree with all functional units needed to create the goal based on the kitchen items
	tree_allPaths = []; # -- list of ALL possible functional units that can be used for making the item.

	for T in environment:
		itemsSeen.append(T);
		index = -1;
		for N in searchNodes:
			if isinstance(N, FOON.Object) and T.equals_functions[hierarchy_level-1](N):
				index = searchNodes.index(N);
		if index > -1:
			# -- this means that the object exists in FOON; if not..
			kitchen.append(searchNodes[index]);
	#endfor
    
	max_iterations = 0; prior = -1; further = -1;
	depth = 100000; # -- maximum number of times you can "see" the original goal node.
    ## start in the algorithm ##
	while itemsToSearch:
		print "length of itemsToSearch %s" %len(itemsToSearch)
		# -- Remove the item we are trying to make from the queue of items we need to learn how to make
		tempObject = itemsToSearch.popleft();
		tempObject.printObject_Lvl3();

		# -- sort-of a time-out for the search if it does not produce an answer dictated by the amount of time it takes.
		print "prior %s" %prior
		print "further %s" %further
		print "length of itemsToSearch %s" %len(itemsToSearch)
		if prior == len(itemsToSearch) and further == prior:
			max_iterations += 1;

		print "max_iterations %s" %max_iterations
		print "depth %s" %depth
		if max_iterations > depth:	# just the worst possible way of doing this, but will do for now.
			return False;

		further = prior;
		print "further %s" %further
		prior = len(itemsToSearch);
		print "prior is %s" %prior

		flag = False;
		# check if goal object in kitchen
		for S in kitchen:
			# print S.printObject_Lvl2()
			# print goalNode.printObject_Lvl2()
			if S.equals_functions[hierarchy_level-1](goalNode):
				print "flag %s" %flag
				flag = True;
				break;
		print "flag is %s" %flag
		if flag == True:
			# -- If we found the item already, why continue searching? (Base case)
			#		therefore we break here!
			break;

		flag = False;
		for S in kitchen:
			print S.printObject_Lvl2()
			print tempObject.printObject_Lvl2()
			if S.equals_functions[hierarchy_level-1](tempObject):
				print "flag is %s" %flag
				flag = True;
				break;
		print "flag is %s" %flag
		if flag == True:
			# just proceed to next iteration, as we already know how to make current item!
			continue;

		numProcedures = 0;
		for FU in searchFOON:
			print FU.printFunctionalUnit()
			# -- searching for all functional units with our goal as output
			found = -1;
			for N in FU.getOutputList():
				print N.printObject_Lvl2()
				print tempObject.printObject_Lvl2()
				if N.equals_functions[hierarchy_level-1](tempObject):
					print "get inside"
					found += 1;
			# -- only add a functional unit if it produces a specific output.
			print "found is %s" %found
			if found > -1:
				FUtoSearch.append(FU);
				tree_allPaths.append(FU);
				numProcedures += 1;
		print "numProcedures is %s" %numProcedures
		# -- this means that there is NO solution to making an object,
		#		and so we just need to add it as something we still need to learn how to make.
		# -- this should probably indicate that there is no task tree.
		if FUtoSearch == False:
			# -- if a solution has not been found yet, add the object back to queue.
			found = False;
			# -- both conditions true -> we have already seen the object and we have it
			for U in kitchen:
				if U.equals_functions[hierarchy_level-1](tempObject):
					found = True;
					break;

			for U in itemsToSearch:
				if U.equals_functions[hierarchy_level-1](tempObject):
					found = True;
					break;

			if found == False:
				print "ready to put"
				itemsToSearch.put(tempObject);

		while FUtoSearch:
			tempFU = FUtoSearch.pop();
			print tempFU.printFunctionalUnit()
			count = 0;
			for T in tempFU.getInputList():
				flag = False;
				for U in kitchen:
					# print U.printObject_Lvl2()
					# print T.printObject_Lvl2()
					if U.equals_functions[hierarchy_level-1](T):
						print "get inside"
						flag = True;
						break;
				print "flag is %s" %flag
				if flag == False:
					# -- if an item is not in the "objects found" list, 
					#		then we add it to the list of items we then need to explore and find out how to make.
					for U in itemsToSearch:
						# print U.printObject_Lvl2()
						# print T.printObject_Lvl2()
						if U.equals_functions[hierarchy_level-1](T):
							print "get inside"
							flag = True;
							break;
					print "flag is %s" %flag
					if flag == False:
						print T.printObject_Lvl2()
						itemsToSearch.append(T);
				else :
					# keeping track of whether we have all items for a functional unit or not!
					count += 1;
			print "count %s" %count
			numProcedures -= 1;
			print "tempFU.getNumberOfInputs %s" %tempFU.getNumberOfInputs()
			print "numProcedures %s" %numProcedures
			if count == tempFU.getNumberOfInputs():
				# We will have all items needed to make something;
				#	add that item to the "kitchen", as we consider it already made.
				found = False;
				for U in kitchen:
					if U.equals_functions[hierarchy_level-1](tempObject):
						found = True;
						break;
				if found == False:
					kitchen.append(tempObject);

				# -- Ensuring that we do not add duplicate objects to these lists..
				found = False;
				for U in itemsSeen:
					if U.equals_functions[hierarchy_level-1](tempObject):
						found = True;
						break;
				if found == False:
					itemsSeen.append(tempObject);

				for x in range(numProcedures):
					# remove all functional units that can make an item - we take the first!
					FUtoSearch.pop();

				found = False;
				for FU in taskTree:
					# FU.printFunctionalUnit()
					# print "----------------"
					# tempFU.printFunctionalUnit()
					if FU.equals_functions[hierarchy_level-1](tempFU):
						# ensuring that we do not repeat any units
						found = True;
						break;
				if found == False:
					taskTree.append(tempFU);
			else:
				# -- if a solution has not been found yet, add the object back to queue.
				found = False;
				# -- both conditions true -> we have already seen the object and we have it
				for U in itemsToSearch:
					print U.printObject_Lvl2()
					print tempObject.printObject_Lvl2()
					if U.equals_functions[hierarchy_level-1](tempObject):
						print "get inside"
						found = True;
						break;
				if found == False:
					print tempObject.printObject_Lvl2()
					itemsToSearch.append(tempObject);

	# -- saving task tree sequence to file..
	# _file = open("FOON_task-tree-for-O" + goalType + "_S" + goalState + "_Lvl" + str(hierarchy_level) + ".txt", 'w');

	# for FU in taskTree:
	# 	# -- just write all functional units that were put into the list:
	# 	FU.printFunctionalUnit();
	# 	_file.write(FU.getInputsForFile());
	# 	_file.write(FU.getMotionForFile());
	# 	_file.write(FU.getOutputsForFile());
	# 	_file.write("//\n");
	#endfor
	return taskTree;
#endef

def get_object_info(task_tree):
	object_id = []
	object_state_id = []
    
	for FU in task_tree:
		tmp = re.split('\t|\n',FU.getInputsForFile())
		if tmp[0].startswith("O"):
			object_id.append(int(tmp[0][1:]))
		if tmp[3].startswith("S"):
			object_state_id.append(tmp[3])
	return list(set(object_id))

totalNodes = constructFUGraph('kitting_experiment_lib.txt'); # -- NOTE: replace text file here with any subgraph/FOON graph file.

# -- Refer to Paulius et al. 2018 paper for explanation on hierarchies:
print " -> number of units in level 3: " + str(len(FOON_Lvl3));
print " -> number of units in level 2: " + str(len(FOON_Lvl2));
print " -> number of units in level 1: " + str(len(FOON_Lvl1));

for F in FOON_Lvl3:
	F.printFunctionalUnit();
	print "//";
# ipdb.set_trace()
# findAllPaths(); # -- use this to find all paths to making an object
task_tree = taskTreeRetrieval(); # -- use this to find a task tree
tracking_objects_list = get_object_info(task_tree)



