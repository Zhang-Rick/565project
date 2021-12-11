import sys

def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False



file_location = str(sys.argv[1])
in_file = file_location + "/stats.txt"
out_file = file_location + "/BP_stats.txt"

f = open(in_file,'r')

list_of_lines = f.readlines()

f.close()



no_of_mispred = [int(s) for s in list_of_lines[18].split() if s.isdigit()][0]
no_of_pred    = [int(s) for s in list_of_lines[19].split() if s.isdigit()][0]
cpi    = [float(s) for s in list_of_lines[117].split() if isfloat(s)][0]

accuracy = (float(no_of_pred) - float(no_of_mispred))/float(no_of_pred)*100

f = open(out_file,'w')

f.write("Accuracy = " + str(accuracy) + "\n")
f.write("Misprediction = " + str(100 - accuracy) + "\n")
f.write("CPI = " + str(cpi) + "\n")

f.close()



#list_of_lines[74] = '    localPredictorSize = Param.Unsigned(' + str(sys.argv[1])  + ', "Size of local predictor")'

#f = open(file_name,'w')

#f.writelines(list_of_lines)

#f.close()

