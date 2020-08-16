#!/bin/bash

EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/Sidelink/sidelink/Release/sidelink"

for NUAV in {2..10..4}
do
	echo "Executing with $NUAV UAVs"
	for DIST in {20..200..40}
	do
		echo "   Executing at $DIST distance"
		for LOAD in {100..1000..100}
		do
			echo "      Executing with load of $LOAD"
			
			$EXEC -time 10000 -nu $NUAV -sPoIdist $DIST -npktint $LOAD > /tmp/ris.log
			
			#echo "${LOAD} ${NUAV} ${DIST} $RIS" >> "out/test_U${NUAV}_D${DIST}.dat"
			#echo "${DIST} ${NUAV} ${LOAD} $RIS" >> "out/test_U${NUAV}_L${LOAD}.dat"
			#echo "${NUAV} ${DIST} ${LOAD} $RIS" >> "out/test_D${DIST}_L${LOAD}.dat"
			
			cat /tmp/ris.log >> "out2/test_U${NUAV}_D${DIST}.dat"
			cat /tmp/ris.log >> "out2/test_U${NUAV}_L${LOAD}.dat"
			cat /tmp/ris.log >> "out2/test_D${DIST}_L${LOAD}.dat"
			
			#exit
		done
	done
done
