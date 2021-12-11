
#python accuracy_calc/edit_branch_predictor.py $1

#./make_arm.sh

./run_arm.sh $1 $2

python accuracy_calc/calculate_accuracy.py $1
