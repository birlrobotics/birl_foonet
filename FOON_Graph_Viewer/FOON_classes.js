class Thing {
    constructor(n, l) {
        this.identifier = n; this.label = l;
    }

    setLabel(S) {
    	this.label = S;
    }

    getLabel() {
    	return this.label;
    }

    getType() {
        return this.identifier;
    }

    setType(T) {
    	this.identifier = T;
    }

    equals(T) {
    	return T.getType() == this.getType();
    }
}

class ObjectNode {

	// -- constructor method for an Object object (lol)
    constructor(N, S, M, L){
    	this.setObjectType(N);
    	this.setObjectState(S);
        this.setObjectLabel(M);
        this.setStateLabel(L);
        this.contained = [];
		this.numIngredients = 0;
		this.print_functions = [this.printObject_Lvl1, this.printObject_Lvl2, this.printObject_Lvl3];
		this.equals_functions = [this.equals_Lvl1, this.equals_Lvl2, this.equals_Lvl3];
    }

    equals_Lvl2(O){
    	return (O.getObjectType() == this.getObjectType() && O.getObjectState() == this.getObjectState());
    }

	equals_Lvl1(O){
		return (O.getObjectType() == this.getObjectType());
	}

    equals_Lvl3(O){
    	return this.equals_Lvl2(O) && this.isSameIngredients(O);
    }

    isSameIngredients(O){
    	var count = 0;
    	for (var I = 0; I < this.contained.length; I++){
			for (var J = 0; J < O.contained.length; J++){
				if (this.contained[I] === (O.contained[J])){
	    			count++;
	    		}
			}
    	}
		if (O.contained.length == 0 && this.contained.length == 0){
			return true;
		}
    	if (count == O.numIngredients && O.contained.length == this.contained.length){
    		return true;
    	}
    	return false;
    }

	getNumberOfIngredients(){
		return this.numIngredients;
	}

    getObjectLabel(){
    	return this.objectLabel;
    }

    getObjectType(){
        return this.objectType;
    }

    getStateLabel(){
    	return this.stateLabel;
    }

	getObjectState(){
        return this.objectState;
    }

    setObjectType(T){
    	this.objectType = T;
    }

    getIngredientsList(){
    	return this.contained;
    }

    setIngredientsList(L){
    	this.contained = L;
    	this.numIngredients = this.contained.length;
    }

    setObjectState(S){
    	this.objectState = S;
    }

    setObjectLabel(S){
    	this.objectLabel = S;
    }

    setStateLabel(S){
    	this.stateLabel = S;
    }

    setIngredient(O){
		var flag = false;
		for (var I = 0; I < this.contained.length; I++) {
			if (this.contained[I] === O) {
				flag = true;
			}
		}
		if (flag == false){
    		this.contained.push(O);
    		this.numIngredients++;
    	}
    }

    getIngredients(){
    	var result = "";
    	if (this.numIngredients === 0){
    		return result;
    	}
    	result = result.concat("{");
    	for (var count = 0; count < this.numIngredients; count++){
			result = result.concat(this.contained[count]);
			if (count < (this.numIngredients - 1)){
				result = result.concat(",");
			}
		}
    	result = result.concat("}");
    	return result;
    }
	
	printObject_Lvl1(){
		console.log("O" + this.objectType + "\t" + this.objectLabel);
	}

	printObject_Lvl2(){
		console.log("O" + this.objectType + "\t" + this.objectLabel);
		console.log("S" + this.objectState + "\t" + this.getStateLabel());
	}

	printObject_Lvl3(){
		console.log("O" + this.objectType + "\t" + this.objectLabel);
		console.log("S" + this.objectState + "\t" + this.getStateLabel() + "\t" + this.getIngredients());
	}
}

class MotionNode {

	constructor(N, L){
        this.setMotionType(N);
        this.setLabel(L);
    }

	setLabel(L){
		this.motionLabel = L;
	}

	getMotionType(){
        return this.motionType;
    }

	getLabel(){
		return this.motionLabel;
	}

	printMotion(){
		console.log("M"+this.getMotionType()+"\t"+this.getLabel());
	}

    equals(M){
    	return M.getMotionType() === this.getMotionType();
    }

	getMotion(){
		var text = "M" + this.getMotionType() + "\t" + this.getMotionLabel(); 
		return text;
	}

    setMotionType(T){
    	this.motionType = T;
    }
}

class FunctionalUnit {

	constructor() {
		this.inputNodes = [];
		this.outputNodes = [];
		this.inDescriptor = [];
		this.outDescriptor = [];
		this.motionNode = new MotionNode();
		this.times = ["", ""];
		this.indication = [];
		this.success_rate = -1.0;
		this.equals_functions = [this.equals_Lvl1, this.equals_Lvl2, this.equals_Lvl3];
		this.print_functions = [this.printFunctionalUnit_Lvl1, this.printFunctionalUnit_Lvl2, this.printFunctionalUnit_Lvl3];
	}

	addObjectNode(O, N, D) {
		if (N == 0) {
			this.inputNodes.push(O);
			this.inDescriptor.push(D);
		} else if (N == 1) {
			this.outputNodes.push(O);
			this.outDescriptor.push(D);
		} else { }
	}

	equals_Lvl1(U){
		var results = 0; // this number must add up to three (3) which suggests that all parts match!
		var count = 0; // counter used to determine number of hits (true matches)
		// checking if the input nodes are all the same!
		for (var T = 0; T < this.inputNodes.length; T++){
			for (var TU = 0; TU < U.inputNodes.length; TU++){
				if (this.inputNodes[T].equals_Lvl1(U.inputNodes[TU])){
					count++;
				}
			}
		}
		// if the counter matches up to the number of inputs,
		//	then that means we have the same set of inputs.
		if (count == this.getNumberOfInputs()){
			results++;
		}

		// checking if the Motion is the same
		if (this.motionNode.equals(U.motionNode)){
			results++;
		}

		// checking if the output nodes are all the same!
		count = 0;
		for (var T = 0; T < this.outputNodes.length; T++){
			for (var TU = 0; TU < U.outputNodes.length; TU++){
				if (this.outputNodes[T].equals_Lvl1(U.outputNodes[TU])){
					count++;
				}
			}
		}
		if (count == this.getNumberOfOutputs()){
			results++;
		}

		// simply return true or false depending on the value of results
		return (results == 3);
	}

	equals_Lvl2(U){
		var results = 0; // this number must add up to three (3) which suggests that all parts match!
		var count = 0; // counter used to determine number of hits (true matches)
		// checking if the input nodes are all the same!
		for (var T = 0; T < this.inputNodes.length; T++){
			for (var TU = 0; TU < U.inputNodes.length; TU++){
				if (this.inputNodes[T].equals_Lvl2(U.inputNodes[TU])){
					count++;
				}
			}
		}
		// if the counter matches up to the number of inputs,
		//	then that means we have the same set of inputs.
		if (count == this.getNumberOfInputs()){
			results++;
		}

		// checking if the Motion is the same
		if ((this.motionNode).equals(U.motionNode)){
			results++;
		}

		// checking if the output nodes are all the same!
		count = 0;
		for (var T = 0; T < this.outputNodes.length; T++){
			for (var TU = 0; TU < U.outputNodes.length; TU++){
				if (this.outputNodes[T].equals_Lvl2(U.outputNodes[TU])){
					count++;
				}
			}
		}
		if (count == this.getNumberOfOutputs()){
			results++;
		}

		// simply return true or false depending on the value of results
		return (results == 3);
	}

	equals_Lvl3(U){
		var results = 0; // this number must add up to three (3) which suggests that all parts match!
		var count = 0; // counter used to determine number of hits (true matches)
		// checking if the input nodes are all the same!
		for (var T = 0; T < this.inputNodes.length; T++){
			for (var TU = 0; TU < U.inputNodes.length; TU++){
				if (this.inputNodes[T].equals_Lvl3(U.inputNodes[TU])){
					count++;
				}
			}
		}
		// if the counter matches up to the number of inputs,
		//	then that means we have the same set of inputs.
		if (count == this.getNumberOfInputs()){
			results++;
		}

		// checking if the Motion is the same
		if ((this.motionNode).equals(U.motionNode)){
			results++;
		}

		// checking if the output nodes are all the same!
		count = 0;
		for (var T = 0; T < this.outputNodes.length; T++){
			for (var TU = 0; TU < U.outputNodes.length; TU++){
				if (this.outputNodes[T].equals_Lvl3(U.outputNodes[TU])){
					count++;
				}
			}
		}
		if (count == this.getNumberOfOutputs()){
			results++;
		}

		// simply return true or false depending on the value of results
		return (results == 3);
	}

	getMotionNode(){
		return this.motionNode;
	}

	getInputList(){
		return this.inputNodes;
	}

	getOutputList(){
		return this.outputNodes;
	}

	setMotionNode(M){
		this.motionNode = M;
	}

	setInputList(L){
		this.inputNodes = L;
	}

	setOutputList(L){
		this.outputNodes = L;
	}

	getNumberOfInputs(){
		return this.inputNodes.length;
	}

	getNumberOfOutputs(){
		return this.outputNodes.length;
	}

	setTimes(S, E){
		this.times[0] = S; this.times[1] = E;
	}

	getStartTime(){
		return this.times[0];
	}

	getEndTime(){
		return this.times[1];
	}

	getSuccessRate(){
		return this.success_rate;
	}

	setSuccessRate(SR){
		this.success_rate = SR;
	}

	getIndication(){
		return this.indication;
	}

	setIndication(I){
		this.indication = I;
	}

	printFunctionalUnit_Lvl1(){
		var count = 0;
		for (T in this.inputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.inDescriptor[count]);
			count++;
		}
		console.log(this.motionNode.getMotion() + "\t" + this.times[0] + "\t" + this.times[1]);
		count = 0;
		for (T in this.outputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.outDescriptor[count]);
			count++;
		}
		if (this.success_rate > -1.0)
			console.log("success rate for Robot: " + this.success_rate);
	}

	printFunctionalUnit_Lvl2(){
		var count = 0;
		for (T in this.inputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.inDescriptor[count]);
			console.log("S" + T.getStateType() + "\t" + T.getStateLabel());
			count++;
		}
		console.log(this.motionNode.getMotion() + "\t" + this.times[0] + "\t" + this.times[1]);
		count = 0;
		for (T in this.outputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.outDescriptor[count]);
			console.log("S" + T.getStateType() + "\t" + T.getStateLabel());
			count++;
		}
		if (this.success_rate > -1.0)
			console.log("success rate for Robot: " + this.success_rate);
	}

	printFunctionalUnit_Lvl3(){
		var count = 0;
		for (T in this.inputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.inDescriptor[count]);
			console.log("S" + T.getStateType() + "\t" + T.getStateLabel() + "\t" + T.getIngredients());
			count = count++
		}
		console.log(this.motionNode.getMotion() + "\t" + this.times[0] + "\t" + this.times[1]);
		count = 0;
		for (T in this.outputNodes){
			console.log("O" + T.getObjectType() + "\t" + T.getObjectLabel() + "\t" + this.outDescriptor[count]);
			console.log("S" + T.getStateType() + "\t" + T.getStateLabel() + "\t" + T.getIngredients());
			count++;
		}
		if (this.success_rate > -1.0)
			console.log("success rate for Robot: " + this.success_rate);
	}
}
