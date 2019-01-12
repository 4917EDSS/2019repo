import json, sys, os
inpath=sys.argv[1]
inpath=os.path.abspath(inpath)
f=open(inpath, "r+")
data=f.read()
f.close()
commands=json.loads(data)
header="class {}".format(commands[0]["name"])
header=header+"{\n"
header=header+"  {}();\n".format(commands[0]["name"])
header=header+"}"
includes=""
includestrs=[]
for com in commands[1] :
    if not com["name"] in includestrs :
        includestrs.append(com["name"])
for i in includestrs :
    includes+="#include \"{}.h\"\n".format(i)
output="{}::{}()".format(commands[0]["name"],commands[0]["name"])
output+=" {\n"
for command in range(len(commands[1])) :
    comstring="  "
    if commands[1][command]["parallel"] :
        comstring+="addParallel"
    else :
        comstring+="addSequential"
    comstring+="(new {}(".format(commands[1][command]["name"])
    for arg in commands[1][command]["args"] :
        comstring+="{},".format(str(arg))
    comstring=comstring[:-1]
    comstring+="));\n"
    output+=comstring
output+="}"
output=includes+output
os.system("touch {}.cpp".format(commands[0]["name"]))
f=open("{}.cpp".format(commands[0]["name"]), "r+")
f.write(output)
f.close()
os.system("touch \"{}.h\"".format(commands[0]["name"]))
f=open("{}.h".format(commands[0]["name"]),"r+")
f.write(header)
f.close()