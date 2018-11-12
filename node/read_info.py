import numpy 

file_name = "info.txt"
_file = open(file_name, 'r')
items = _file.read().splitlines()
phase_count = 0
object_id = None
state_id = None

object_info = []

x = 0
while x < len(items):
    line = items[x]
    if line.startswith("//"):
        phase_count += 1
    elif line.startswith("O"):
        objectParts = line.split("O")
        objectParts = objectParts[1].split("\t")
        object_id = int(objectParts[0])

        x += 1
        line = items[x]
        stateParts = line.split("S"); # get the Object's state identifier by splitting first instance of S
        stateParts = stateParts[1].split("\t");
        state_id = int(stateParts[0])
   
        state_dic = {"phase":phase_count,
                     "object_id":object_id,
                     "state_id":state_id,}

    object_info.append(state_dic) 
    x += 1
print object_info