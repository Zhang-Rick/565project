import os
import re
filename = os.listdir('prophet')
filename2 = os.listdir('revised original')
print(filename)
for element in filename:
    if element.split('-')[2] == '2bit.txt':
        with open('prophet/'+element) as f:
            lines = f.read()
            #print(lines)
        pattern1 =r'system.cpu.branchPred.condIncorrect[ \t]+[0-9]+'
        pattern2 = r'system.cpu.branchPred.condPredicted[ \t]+[0-9]+'
        pattern3 = r'system.cpu.cpi[ \t]+[0-9]+[.]*[0-9]*'
        conIncorrect = re.search(pattern1,lines)
        condPredicted = re.search(pattern2,lines)
        cpi = re.search(pattern3,lines)
        conIncorrectInt = int(conIncorrect[0].split(' ')[-1])
        condPredictedInt = int(condPredicted[0].split(' ')[-1])
        accuracy = (1 - conIncorrectInt/condPredictedInt)*100
        cpiInt = cpi[0].split(' ')[-1]
        print(cpiInt, element)

print('reviced')
for element in filename2:
    with open('revised original/'+element) as f:
        lines = f.read()
        #print(lines)
    pattern1 =r'system.cpu.branchPred.condIncorrect[ \t]+[0-9]+'
    pattern2 = r'system.cpu.branchPred.condPredicted[ \t]+[0-9]+'
    pattern3 = r'system.cpu.cpi[ \t]+[0-9]+[.]*[0-9]*'
    conIncorrect = re.search(pattern1,lines)
    condPredicted = re.search(pattern2,lines)
    cpi = re.search(pattern3,lines)
    conIncorrectInt = int(conIncorrect[0].split(' ')[-1])
    condPredictedInt = int(condPredicted[0].split(' ')[-1])
    accuracy = (1 - conIncorrectInt/condPredictedInt)*100
    cpiInt = cpi[0].split(' ')[-1]
    print(cpiInt, element)#,element)

