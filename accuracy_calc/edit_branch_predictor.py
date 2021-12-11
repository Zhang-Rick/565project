import sys

file_name = "/home/min/a/anand109/ECE565/pa1-anand109purdue/src/cpu/pred/BranchPredictor.py"

f = open(file_name,'r')

list_of_lines = f.readlines()

f.close()

list_of_lines[97] = '    h = Param.Unsigned(' + str(sys.argv[1])  + ', "h")\n'
f = open(file_name,'w')

f.writelines(list_of_lines)

f.close()
