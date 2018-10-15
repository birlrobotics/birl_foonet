# NOTE: This file contains the classes needed for the FOON graph programs.

# -- Last updated: 6/23/2017

class Thing(object):
	# -- A thing is a general node; it can be either an object or a motion node.
	# -- Thing objects have three (3) elements:
	#		1. an identifier (ID), which is an integer,
	#		2. a string label, which tells the user what type of Thing it is,
	#		3. a list of other Things that it is connected to via an edge.

	# -- constructor methods for creating Thing object:
	def __init__(self, T=None, L=None):
		self.type = T;
		self.label = L;
		self.neighbours = [];
	#enddef

	# -- accessor methods for Things:
	def getType(self):
		return self.type;

	def getLabel(self):
		return self.label;

	def getNeigbourList(self):
		return self.neighbours;

	def setType(self, T):
		self.type = T;

	def setLabel(self, L):
		self.label = L;

	def addNeighbour(self, N):
		self.neighbours.append(N);

	def countNeighbours(self):
		return len(self.neighbours);

	def printThing(self):
		if isinstance(self, Object):
			print self.printObject();
		elif isinstance(self, Motion):
			print self.printMotion();
		else:
			print "O" + str(self.type) + "\t" + self.label;

	def equals(self, T):
		return self.type == T.getType();

class Object(Thing):
	def __init__(self, N=None, S=None, M=None, L=None):
		super(Object,self).__init__(N, M);
		self.objectState = S;
		self.stateLabel = L;
		self.contained = [];
		self.numIngredients = 0;
		self.equals_functions = [self.equals_Lvl1, self.equals_Lvl2, self.equals_Lvl3];
	#enddef

	# -- accessor methods for Things:
	def getObjectType(self):
		return super(Object,self).getType();

	def setObjectType(self, N):
		super(Object,self).setType(N);

	def getObjectLabel(self):
		return super(Object,self).getLabel();

	def setObjectLabel(self, L):
		super(Object,self).setLabel(L);

	def getStateType(self):
		return self.objectState;

	def setStateType(self, S):
		self.objectState = S;

	def getStateLabel(self):
		return self.stateLabel;

	def setStateLabel(self, L):
		return self.stateLabel;

	def getIngredientsList(self):
		return self.contained;

	def addIngredient(self, I):
		if I not in self.contained:
			self.contained.append(I);
			self.numIngredients += 1;

	def getIngredients(self):
		ingredients = "";
		if self.numIngredients == 0:
			return ingredients;
		ingredients = ingredients + "{";
		for x in range(len(self.contained)):
			ingredients = ingredients + self.contained[x];
			if x < (self.numIngredients - 1):
				ingredients = ingredients + ","
			#endif
		#endfor
		ingredients = ingredients + "}";
		return ingredients;

	def printObject_Lvl2(self):
		print "O" + str(self.getType()) + "\t" + self.getLabel();
		print "S" + str(self.getStateType()) + "\t" + self.getStateLabel();

	def printObject_Lvl3(self):
		print "O" + str(self.getType()) + "\t" + self.getLabel();
		print "S" + str(self.getStateType()) + "\t" + self.getStateLabel() + "\t" + self.getIngredients();

	def isSameIngredients(self, O):
		count = 0;
		otherList = O.getIngredientsList();
		for x in range(len(otherList)):
			if otherList[x] in self.contained:
				count += 1;
		if count == len(otherList):
			return True;
		else:
			return False;
	#enddef

	def equals_Lvl1(self, O):
		return self.getObjectType() == O.getObjectType();

	def equals_Lvl2(self, O):
		return self.getObjectType() == O.getObjectType() and self.getStateType() == O.getStateType();

	def equals_Lvl3(self, O):
		return self.equals_Lvl2(O) and self.isSameIngredients(O);

class Motion(Thing):
	def __init__(self, N=None, M=None):
		super(Motion,self).__init__(N, M);

	def getMotionType(self):
		return super(Motion,self).getType();

	def setMotionType(self, M):
		super(Motion,self).setType(M);

	def getMotionLabel(self):
		return super(Motion,self).getLabel();

	def setMotionLabel(self, M):
		super(Motion,self).setLabel(M);

	def equals(self, M):
		return self.getMotionType() == M.getMotionType();

	def printMotion(self):
		print "M" + str(self.getMotionType()) + "\t" + this.getMotionLabel();

	def getMotion(self):
		text = "M" + str(self.getMotionType()) + "\t" + self.getMotionLabel();
		return text;

class FunctionalUnit(object):
	def __init__(self):
		self.inputNodes = [];
		self.outputNodes = [];
		self.inDescriptor = [];
		self.outDescriptor = [];
		self.startTime = "";
		self.endTime = "";
		self.motionNode = Motion();
		self.equals_functions = [self.equals_Lvl1, self.equals_Lvl2, self.equals_Lvl3];
	#enddef

	def addObjectNode(self, O, N, D):
		if N == 1:
			self.inputNodes.append(O);
			self.inDescriptor.append(D);
		elif N == 2:
			self.outputNodes.append(O);
			self.outDescriptor.append(D);
		else:
			pass;
	#enddef

	def equals_Lvl3(self, U):
		results = 0; 	# -- this number must add up to three (3) which suggests that all parts match!
		count = 0; 		# -- counter used to determine number of hits (true matches)

		# 1. Check if the input nodes are all the same:
		for T in self.inputNodes:
			for TU in U.inputNodes:
				if T.equals_Lvl3(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfInputs():
			results = results + 1;

		# 2. Check if the Motion is the same
		if self.motionNode.equals(U.motionNode):
			results = results + 1;

		# 3. Check if the output nodes are all the same:
		for T in self.outputNodes:
			for TU in U.outputNodes:
				if T.equals_Lvl3(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfOutputs():
			results = results + 1;

		# -- simply return true or false depending on the value of results
		return results == 3;
	#enddef

	def equals_Lvl2(self, U):
		results = 0; 	# -- this number must add up to three (3) which suggests that all parts match!
		count = 0; 		# -- counter used to determine number of hits (true matches)

		# 1. Check if the input nodes are all the same:
		for T in self.inputNodes:
			for TU in U.inputNodes:
				if T.equals_Lvl2(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfInputs():
			results = results + 1;

		# 2. Check if the Motion is the same
		if self.motionNode.equals(U.motionNode):
			results = results + 1;

		# 3. Check if the output nodes are all the same:
		for T in self.outputNodes:
			for TU in U.outputNodes:
				if T.equals_Lvl2(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfOutputs():
			results = results + 1;

		# -- simply return true or false depending on the value of results
		return results == 3;
	#enddef

	def equals_Lvl1(self, U):
		results = 0; 	# -- this number must add up to three (3) which suggests that all parts match!
		count = 0; 		# -- counter used to determine number of hits (true matches)

		# 1. Check if the input nodes are all the same:
		for T in self.inputNodes:
			for TU in U.inputNodes:
				if T.equals_Lvl1(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfInputs():
			results = results + 1;

		# 2. Check if the Motion is the same
		if self.motionNode.equals(U.motionNode):
			results = results + 1;

		# 3. Check if the output nodes are all the same:
		for T in self.outputNodes:
			for TU in U.outputNodes:
				if T.equals_Lvl1(TU):
					count = count + 1;

		# -- if the counter matches up to the number of inputs,
		#		then that means we have the same set of inputs.
		if count == self.getNumberOfOutputs():
			results = results + 1;

		# -- simply return true or false depending on the value of results
		return results == 3;
	#enddef

	def getMotion(self):
		return self.motionNode;

	def setMotion(self, M):
		self.motionNode = M;

	def getInputList(self):
		return self.inputNodes;

	def getOutputList(self):
		return self.outputNodes;

	def getInputDescriptor(self):
		return self.inDescriptor;

	def getOutputDescriptor(self):
		return self.outDescriptor;

	def setInputList(self, L):
		self.inputNodes = L;

	def setOutputList(self, L):
		self.outputNodes = L;

	def getNumberOfInputs(self):
		return len(self.inputNodes);

	def getNumberOfOutputs(self):
		return len(self.outputNodes);

	def setTimes(self, S, E):
		self.startTime = S;
		self.endTime = E;

	def getStartTime(self):
		return self.startTime;

	def getEndTime(self):
		return self.endTime;

	def printFunctionalUnit(self):
		count = 0;
		for T in self.inputNodes:
			print "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel() + "\t" + str(self.inDescriptor[count]);
			print "S" + str(T.getStateType()) + "\t" + T.getStateLabel() + "\t" + T.getIngredients();
			count = count + 1;
		#endfor
		print self.motionNode.getMotion() + "\t" + self.startTime + "\t" + self.endTime;
		count = 0;
		for T in self.outputNodes:
			print "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel() + "\t" + str(self.outDescriptor[count]);
			print "S" + str(T.getStateType()) + "\t" + T.getStateLabel() + "\t" + T.getIngredients();
			count = count + 1;
		#endfor

	def printFunctionalUnit_Lvl2(self):
		count = 0;
		for T in self.inputNodes:
			print "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel() + "\t" + str(self.inDescriptor[count]);
			print "S" + str(T.getStateType()) + "\t" + T.getStateLabel();
			count = count + 1;
		#endfor
		print self.motionNode.getMotion() + "\t" + self.startTime + "\t" + self.endTime;
		count = 0;
		for T in self.outputNodes:
			print "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel() + "\t" + str(self.outDescriptor[count]);
			print "S" + str(T.getStateType()) + "\t" + T.getStateLabel();
			count = count + 1;
		#endfor

	def getInputsForFile(self):
		cat = "";
		count = 0;
		for T in self.inputNodes:
			# -- just keep adding all Strings which describe all Objects and then return
			cat = cat + "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel();
			cat = cat + "\t" + str(self.inDescriptor[count]) + "\n";
			cat = cat + "S" + str(T.getStateType()) + "\t" + T.getStateLabel() + "\t" + T.getIngredients() + "\n";
			count = count + 1;
		return cat;

	def getOutputsForFile(self):
		cat = "";
		count = 0;
		for T in self.outputNodes:
			# -- just keep adding all Strings which describe all Objects and then return
			cat = cat + "O" + str(T.getObjectType()) + "\t" + T.getObjectLabel();
			cat = cat + "\t" + str(self.outDescriptor[count]) + "\n";
			cat = cat + "S" + str(T.getStateType()) + "\t" + T.getStateLabel() + "\t" + T.getIngredients() + "\n";
			count = count + 1;
		return cat;

	def getMotionForFile(self):
		return self.motionNode.getMotion() + "\t" + self.startTime + "\t" + self.endTime + "\n";

	def inputExists(self, I):
		for T in self.inputNodes:
			if I.equals(T):
				 return True;
		return False;

	def outputExists(self, I):
		for T in self.outputNodes:
			if I.equals(T):
				 return True;
		return False;
