cat settings.py
set -x
for i in 31 46 5 13 21 31 77 7 6 21 24 85 93 86 58 0 68 4 95 51; 
    do  echo "random_object"$i; 
	cat ../domains/robotics_autogen_prob_100objs_template.pddl |sed s/"random_object8 Table6"/"random_object"$i" Table6"/g > ../domains/robotics_autogen_prob.pddl;
	(/usr/bin/time -f "%e real, %U user, %S sys" python ./hybridPlanner.py|grep -v cpp|grep -v odecollision) &>>./outputs/test$i.out; 
	printf "********************\n\n\n\n">> ./outputs/test$i.out; 
    done; 

