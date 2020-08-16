#!/bin/bash

EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/Sidelink/sidelink/Release/sidelink"

for NUAV in {2..10..2}
do
	echo "Executing with $NUAV UAVs"
	for DIST in {50..500..50}
	do
		echo "   Executing at $DIST distance"
		for LOAD in {10..100..10}
		do
			echo "      Executing with load of $LOAD"
			
			RIS=`$EXEC -time 30000 -nu $NUAV -sPoIdist $DIST -npktint $LOAD`
			echo "${NUAV} ${DIST} $RIS" >> "out/test_U${NUAV}_D${DIST}.dat"
			echo "${NUAV} ${LOAD} $RIS" >> "out/test_U${NUAV}_L${LOAD}.dat"
			echo "${DIST} ${LOAD} $RIS" >> "out/test_D${DIST}_L${LOAD}.dat"
		done
	done
done
