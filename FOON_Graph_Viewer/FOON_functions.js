function init (G) {

	var totalNodes = 0; // -- total number of nodes that are in the network
	var FOON_graph = G;

	var nodes_Lvl1 = []; var nodes_Lvl2 = []; var nodes_Lvl3 = [];
  	var FOON_Lvl1 = []; var FOON_Lvl2 = []; var FOON_Lvl3 = [];

	function FUExists(U, A){
		if (A == 1){
			if (FOON_Lvl1.length == 0){
				return false;
			}
			for (var F = 0; F < FOON_Lvl1.length; F++){
				if (FOON_Lvl1[F].equals_Lvl1(U)){
					return true;
				}
			}
			return false;
		}
		if (A == 2){
			if (FOON_Lvl2.length == 0){
				return false;
			}
			for (var F = 0; F < FOON_Lvl2.length; F++){
				if (FOON_Lvl2[F].equals_Lvl2(U)){
					return true;
				}
			}
			return false;
		}
		if (A == 3){
			if (FOON_Lvl3.length == 0){
				return false;
			}
			for (var F = 0; F < FOON_Lvl3.length; F++){
				if (FOON_Lvl3[F].equals_Lvl3(U)){
					return true;
				}
			}
			return false;
		}
	}

	function constructFUGraph() {
		var count = totalNodes; // 'totalNodes' gives an indication of the number of object AND motion nodes are in FOON.
		var stateParts, objectParts, motionParts; // objects used to contain the split strings

		var objectIndex = -1; // variables to hold position of object/motion within list of Things
		var isInput = true;
		var newFU_Lvl3 = new FunctionalUnit(); var newFU_Lvl2 = new FunctionalUnit(); var newFU_Lvl1 = new FunctionalUnit(); // object which will hold the functional unit being read.

		for(var I = 0; I < FOON_graph.length; I++){
			var objectExisting = -1;
			var line = FOON_graph[I];

			if (line.startsWith("//")) {
				if (!FUExists(newFU_Lvl1,1)){
					nodes_Lvl1.push(newFU_Lvl1.getMotionNode());	// no matter what, we add new MotionNode nodes; we will have multiple instances everywhere.
					FOON_Lvl1.push(newFU_Lvl1);
				}
				if (!FUExists(newFU_Lvl2,2)){
					FOON_Lvl2.push(newFU_Lvl2);
					nodes_Lvl2.push(newFU_Lvl2.getMotionNode());	// no matter what, we add new MotionNode nodes; we will have multiple instances everywhere.
				}
				if (!FUExists(newFU_Lvl3,3)){
					nodes_Lvl3.push(newFU_Lvl3.getMotionNode());	// no matter what, we add new MotionNode nodes; we will have multiple instances everywhere.
					FOON_Lvl3.push(newFU_Lvl3);
					count++; // increment number of nodes by one since we are adding a new MotionNode node
				}
				// -- we are adding a new FU, so start from scratch..
				newFU_Lvl3 = new FunctionalUnit(); newFU_Lvl2 = new FunctionalUnit(); newFU_Lvl1 = new FunctionalUnit(); // create an entirely new FU object to proceed with reading new units.
				isInput = true; // this is the end of a FU so we will now be adding input nodes; set flag to TRUE.
			} else if (line.startsWith("O")) {
				// -- this is an Object node, so we probably should read the next line one time
				objectParts = line.split("O", 2); // get the Object identifier by splitting first instance of O
				objectParts = objectParts[1].split("\t");

				// -- read the next line containing the object's state information:
				line = FOON_graph[++I];
				stateParts = line.split("S", 2); // get the Object's state identifier by splitting first instance of S
				stateParts = stateParts[1].split("\t").filter(Boolean);

			// Functional Unit - Level 3:
				var newObject = new ObjectNode(objectParts[0], stateParts[0], objectParts[1], stateParts[1]);

				// -- check if this object is a container:
				if (stateParts.length > 2 && stateParts[2].startsWith("{")){
					var ingredients = stateParts[2];
					ingredients = ingredients.split("{");
					ingredients = ingredients[1].split("}");
					// -- we then need to make sure that there are ingredients to be read!
					if (ingredients.length > 0){
						ingredients = ingredients[0].split(",");
						for (var J = 0; J < ingredients.length; J++){
							newObject.setIngredient(ingredients[J]);
						}
					}
				}

				// -- checking if the Object node exists in the list of objects:
				for (var N = 0; N < nodes_Lvl3.length; N++) {
					if (nodes_Lvl3[N] instanceof ObjectNode && (nodes_Lvl3[N]).equals_Lvl3(newObject)){
						objectExisting = nodes_Lvl3.indexOf(nodes_Lvl3[N]);
					}
				}

				// Check if object already exists within the list so as to avoid duplicates
				if (objectExisting != -1){
					objectIndex = objectExisting;
				}
				else {
					// just add new ObjectNode to the list of all nodes
					nodes_Lvl3.push(newObject);
					objectIndex = count++;
				}

				if (isInput){
					// this Object will be an input node to the FU
					newFU_Lvl3.addObjectNode(nodes_Lvl3[objectIndex], 0, objectParts[2]);
				} else {
					// add the Objects as output nodes to the Functional Unit
					newFU_Lvl3.addObjectNode(nodes_Lvl3[objectIndex], 1, objectParts[2]);
				}

			// Functional Unit - Level 2:
				objectExisting = -1;
				newObject = new ObjectNode(objectParts[0], stateParts[0], objectParts[1], stateParts[1]);

				// checking if Object node exists in the list of objects
				for (var N = 0; N < nodes_Lvl2.length; N++) {
					if (nodes_Lvl2[N] instanceof ObjectNode && (nodes_Lvl2[N]).equals_Lvl2(newObject)){
						objectExisting = nodes_Lvl2.indexOf(nodes_Lvl2[N]);
					}
				}

				// Check if object already exists within the list so as to avoid duplicates
				if (objectExisting != -1){
					objectIndex = objectExisting;
				}
				else {
					// just add new ObjectNode to the list of all nodes
					objectIndex = nodes_Lvl2.length;
					nodes_Lvl2.push(newObject);
				}

				if (isInput){
					// this Object will be an input node to the FU
					newFU_Lvl2.addObjectNode(nodes_Lvl2[objectIndex], 0, objectParts[2]);
				} else {
					// add the Objects as output nodes to the Functional Unit
					newFU_Lvl2.addObjectNode(nodes_Lvl2[objectIndex], 1, objectParts[2]);
				}

			// Functional Unit - Level 1:
				objectExisting = -1;
				var noState = new ObjectNode(objectParts[0], "", objectParts[1], "");

				// checking if Object node exists in the list of objects
				for (var N = 0; N < nodes_Lvl1.length; N++) {
					if (nodes_Lvl1[N] instanceof ObjectNode && nodes_Lvl1[N].equals_Lvl1(noState)){
						objectExisting = nodes_Lvl1.indexOf(nodes_Lvl1[N]);
					}
				}

				// Check if object already exists within the list so as to avoid duplicates
				if (objectExisting != -1){
					objectIndex = objectExisting;
				}
				else {
					// just add new ObjectNode to the list of all nodes
					objectIndex = nodes_Lvl1.length;
					nodes_Lvl1.push(noState);
				}

				if (isInput){
					// this Object will be an input node to the FU
					newFU_Lvl1.addObjectNode(nodes_Lvl1[objectIndex], 0, objectParts[2]);
				} else {
					// add the Objects as output nodes to the Functional Unit
					newFU_Lvl1.addObjectNode(nodes_Lvl1[objectIndex], 1, objectParts[2]);
				}

			} else if (line.startsWith("M")) {
				// We are adding a Motion node, so very easy to deal with
				isInput = false;
				motionParts = line.split("M", 2); // get the Motion number
				motionParts = motionParts[1].split("\t"); // get the Motion label

				// -- FUNCTIONAL UNITS WITH INGREDIENTS...
				// create new MotionNode based on what was read.
				var newMotion = new MotionNode(motionParts[0], motionParts[1]);
				newFU_Lvl3.setMotionNode(newMotion);
				newFU_Lvl3.setTimes(motionParts[2], motionParts[3]);
				if (motionParts.length > 4){
					newFU_Lvl3.setIndication(motionParts[4]);
					newFU_Lvl3.setSuccessRate(motionParts[5]);
				}

				// -- FUNCTIONAL UNITS WITHOUT INGREDIENTS...
				newMotion = new MotionNode(motionParts[0], motionParts[1]);
				newFU_Lvl2.setMotionNode(newMotion);
				newFU_Lvl2.setTimes(motionParts[2], motionParts[3]);
				if (motionParts.length > 4){
					newFU_Lvl2.setIndication(motionParts[4]);
					newFU_Lvl2.setSuccessRate(motionParts[5]);
				}

				// -- FUNCTIONAL UNITS WITHOUT STATES/INGREDIENTS...
				newMotion = new MotionNode(motionParts[0], motionParts[1]);
				newFU_Lvl1.setMotionNode(newMotion);
				newFU_Lvl1.setTimes(motionParts[2], motionParts[3]);
				if (motionParts.length > 4){
					newFU_Lvl1.setIndication(motionParts[4]);
					newFU_Lvl1.setSuccessRate(motionParts[5]);
				}
			}
			else { }
		}
		return count;
	}

	totalNodes = constructFUGraph();

	console.log("FOON Graph constructed!\nLevel 3 Graph contains " + nodes_Lvl3.length + " nodes!");
	displayFOON(FOON_Lvl3);
	return totalNodes;
}

function displayFOON(FOON){
	// -- reference: http://stackoverflow.com/questions/36856232/write-add-data-in-json-file-using-node-js
	var obj = {
		nodes: [],
		links: []
	};

	count = 0;
	for (FU = 0; FU < FOON.length; FU++){
		// -- adding the nodes section of the JSON file..
		tempList = FOON[FU].getInputList();
		for (U = 0; U < tempList.length; U++){
			var objectName = tempList[U].getObjectLabel() + " (" + tempList[U].getStateLabel() + ") " + tempList[U].getIngredients();
			found = false;
			for (V = 0; V < obj.nodes.length; V++){
				if ( obj.nodes[V].id === objectName){
					found = true;
				}
			}
			if (found == false){
				obj.nodes.push({id : objectName, type : 2});
			}
		}
		tempList = FOON[FU].getOutputList();
		for (U = 0; U < tempList.length; U++){
			var objectName = tempList[U].getObjectLabel() + " (" + tempList[U].getStateLabel() + ") " + tempList[U].getIngredients();
			found = false;
			for (V = 0; V < obj.nodes.length; V++){
				if ( obj.nodes[V].id === objectName){
					found = true;
				}
			}
			if (found == false){
				obj.nodes.push({id : objectName, type : 2});
			}
		}
		var motionName = (count) + "_" + FOON[FU].getMotionNode().getLabel();
		console.log(FOON[FU].getSuccessRate());
		if (FOON[FU].getSuccessRate() > -1.0)
			motionName += " (SR: " + String(FOON[FU].getSuccessRate() + ")");
		console.log(motionName);
		obj.nodes.push({id : motionName, type : 1});

		tempList = FOON[FU].getInputList();
		for (U = 0; U < tempList.length; U++){
			var src = tempList[U].getObjectLabel() + " (" + tempList[U].getStateLabel() + ") " + tempList[U].getIngredients();
			var tgt = (count) + "_" + FOON[FU].getMotionNode().getLabel();
			if (FOON[FU].getSuccessRate() > -1.0)
				tgt += " (SR: " + String(FOON[FU].getSuccessRate() + ")");
			obj.links.push({source : src, target : tgt});
		}
		tempList = FOON[FU].getOutputList();
		for (U = 0; U < tempList.length; U++){
			var tgt = tempList[U].getObjectLabel() + " (" + tempList[U].getStateLabel() + ") " + tempList[U].getIngredients();
			var src = (count) + "_" + FOON[FU].getMotionNode().getLabel();
			if (FOON[FU].getSuccessRate() > -1.0)
				src += " (SR: " + String(FOON[FU].getSuccessRate() + ")");
			obj.links.push({source : src, target : tgt});
		}
		count++;
	}

	var json = JSON.stringify(obj);
	console.log("Task tree denoted as:\n"+json);
	drawD3Graph(obj);
	console.log("Graph displayed!");
}

function drawD3Graph (graph) {
	var svg = d3.select("svg").attr("align","center");

	var width = svg.attr("width");
	var height = svg.attr("height");

	svg = svg.call(d3.zoom()
		.on("zoom", zoomed))
		.append("g");

	svg.append("defs")
		.append("marker")
		.attr("id", "arrow")
		.attr("viewBox", "0 -5 10 10")
		.attr("refX", 20)
		.attr("refY", 0)
		.attr("markerWidth", 8)
		.attr("markerHeight", 8)
		.attr("orient", "auto")
		.append("svg:path")
		.attr("d", "M0,-5L10,0L0,5");

	var simulation = d3.forceSimulation()
		.force("link", d3.forceLink().id(function(d) { return d.id; }))
		.force("charge", d3.forceManyBody().strength(-250))
		.force("center", d3.forceCenter(width / 2, height / 2));

	var link = svg.append("g")
		.attr("class", "links")
		.selectAll("line")
		.data(graph.links)
		.enter().append("line")
		.attr("stroke", "black")
		.attr("marker-end", "url(#arrow)");

	var node = svg.append("g")
		.attr("class", "nodes")
		.selectAll("rect")
		.data(graph.nodes)
		.enter()
		.append("circle")
		.attr("r", function(d) { if (d.type == "1") return 14; return 10;})
		.style("fill", function(d) { if (d.type == "1") return "red"; return "#adff2f"; })
		.call(d3.drag()
			.on("start", dragstarted)
			.on("drag", dragged)
			.on("end", dragended))
			.on("mouseover", fade(.08))
			.on("mouseout", fade(1)
		);

	var text = svg.append("g")
		.attr("class", "labels")
		.selectAll("g")
		.data(graph.nodes)
		.enter().append("g");

	text.append("text")
		.attr("x", 14)
		.attr("y", ".29em")
		.style("font-family", "sans-serif")
		.style("font-weight", "bold")
		.style("font-size", "0.8em")
		.text(function(d) { return d.id; });

	node.on("click",function(d){
		console.log("clicked", d.id);
	});

	node.append("title")
		.text(function(d) { return d.id; });

	simulation
		.nodes(graph.nodes)
		.on("tick", ticked);

	simulation.force("link")
		.links(graph.links);


	var linkedByIndex = {};

	graph.links.forEach(function(d) {
		linkedByIndex[d.source.index + "," + d.target.index] = 1;
	});

	function isConnected(a, b) {
		return linkedByIndex[a.index + "," + b.index] || linkedByIndex[b.index + "," + a.index] || a.index == b.index;
	}

	function ticked() {
		link
			.attr("x1", function(d) { return d.source.x; })
			.attr("y1", function(d) { return d.source.y; })
			.attr("x2", function(d) { return d.target.x; })
			.attr("y2", function(d) { return d.target.y; });

		node
			.attr("cx", function(d) { return d.x; })
			.attr("cy", function(d) { return d.y; });

		text
			.attr("transform", function(d) { return "translate(" + d.x + "," + d.y + ")"; })
	}


	function dragstarted(d) {
		if (!d3.event.active)
			simulation.alphaTarget(0.3).restart();
		d.fx = d.x;
		d.fy = d.y;
	}

	function dragged(d) {
		d.fx = d3.event.x;
		d.fy = d3.event.y;
	}

	function dragended(d) {
		if (!d3.event.active)
			simulation.alphaTarget(0);
		d.fx = null;
		d.fy = null;
	}

	function zoomed() {
		svg.attr("transform", "translate(" + d3.event.transform.x + "," + d3.event.transform.y + ")" + " scale(" + d3.event.transform.k + ")");
	}

	function fade(opacity) {
		return function(d) {
			node.style("stroke-opacity", function(o) {
				thisOpacity = isConnected(d, o) ? 1 : opacity;
				this.setAttribute('fill-opacity', thisOpacity);
				return thisOpacity;
			});

			link.style("stroke-opacity", opacity).style("stroke-opacity", function(o) {
				return o.source === d || o.target === d ? 1 : opacity;
			});
		};
	}
}

function clear(){
	d3.selectAll("svg > *").remove();
}
