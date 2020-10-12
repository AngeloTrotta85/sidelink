#!/bin/bash

EXEC="/media/angelo/BigDisk/Programs/Eclipse/newworkspaces/sidelink/Release/sidelink"
INFOLDER="test8ott"
OUTFOLDER="test8ott/Angelo"

NUAV=10
NSF=20
INT=0

#for DIST in {20..200..40}
for CHAN in 2 3 4 5
do
	#echo "   Executing at $DIST distance"
	#for LOAD in {100..1000..100}
	for DR in {100..500..100}
	#for DR in 25 50 75 100 150
	do
		for RUN in {1..5..1}
		do
			for INT in 0 1
			do
				BASICFN="static_pos_10UAV_${CHAN}CH_${DR}DR_R${RUN}"
				FILENAME="${INFOLDER}/${BASICFN}.txt"
				echo "$FILENAME"
				
				$EXEC -time 23999 -timeS 20000 -conf ${FILENAME} -nsf 20 -trace trace/${BASICFN}-NF${NSF}-I${INT}.0.trace -at 0 > ${OUTFOLDER}/${BASICFN}-NF${NSF}-I${INT}.0.log
				$EXEC -time 23999 -timeS 20000 -conf ${FILENAME} -nsf 20 -trace trace/${BASICFN}-NF${NSF}-I${INT}.1.trace -at 1 > ${OUTFOLDER}/${BASICFN}-NF${NSF}-I${INT}.1.log
			done
		done
	done
done

INT=0
CHAN=4
DR=500
for NSF in {10..100..10}
do
	for RUN in {1..5..1}
	do
		FILENAME="${INFOLDER}/static_pos_10UAV_${CHAN}CH_${DR}DR_R${RUN}.txt"
		echo "$FILENAME"
		
		$EXEC -time 23999 -timeS 20000 -conf ${FILENAME} -nsf ${NSF} -trace trace/${BASICFN}-NF${NSF}-I${INT}.0.trace -at 0 > ${OUTFOLDER}/${BASICFN}-NF${NSF}-I${INT}.0.log
		$EXEC -time 23999 -timeS 20000 -conf ${FILENAME} -nsf ${NSF} -trace trace/${BASICFN}-NF${NSF}-I${INT}.1.trace -at 1 > ${OUTFOLDER}/${BASICFN}-NF${NSF}-I${INT}.1.log
		
	done
done

