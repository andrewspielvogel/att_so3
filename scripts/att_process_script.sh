#!/bin/bash

process_att(){

    ATT_SO3="$(rospack find att_so3)"

    mkdir -p $2/proc/microstrain/configs
    mkdir -p $2/proc/microstrain/processed
    mkdir -p $2/proc/microstrain/pdfs
    
    MST=$2/microstrain/$3.MST
    CSV=$2/proc/microstrain/processed/$1.csv
    CONFIG=$2/proc/microstrain/configs/$1.m
    PDF=$2/proc/microstrain/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS

    python $ATT_SO3/python/gen_config_file.py -i $MST -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_mag $8 --k_acc_bias $9 --k_ang_bias ${10} --k_mag_bias ${11} --acc_bias ${12} --ang_bias ${13} --mag_bias ${14} --k_g ${15} --k_north ${16}

    rosrun att_so3 mems_process $CONFIG

    python $ATT_SO3/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}



DIR=/home/lcsr/cruise_data/sentry492
LOG=2018_08_25_01_00
EXP=${LOG}_exp1
HZ=500
rpy_align=[0.0631,0.0044,3.1413]
rpy_ro=[0,0,0]
k_acc=[.1,.1,.1]
k_mag=[.1,.1,.1]
k_acc_bias=[1,1,1]
k_acc_bias=[0,0,0]
k_ang_bias=[0.00001,0.00001,0.00001]
#k_ang_bias=[0,0,0]
k_mag_bias=[.25,.25,0]
#k_mag_bias=[0,0,0]
acc_bias=[0,0,0]
ang_bias=[0,0,0]
mag_bias=[0,0,0]
k_g=[1,1,1]
k_north=[1,1,1]


process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_mag $k_acc_bias $k_ang_bias $k_mag_bias $acc_bias $ang_bias $mag_bias $k_g $k_north








ORD_MAG=10
for w in `seq 1 $ORD_MAG`;
do
    ka=1
    for j in `seq 2 $w`
    do

	ka=0$ka
	
    done
	ka=0.$ka
    for i in `seq 1 $ORD_MAG`;
    do

	UND=_
	EXP=exp_gain_$w$UND$i
	a=$((10**(($i-1))))

	kn=1
	for j in `seq 2 $i`
	do

	    kn=0$kn
	
	done
	kn=0.$kn

	K=[0.1,0.1,$ka,$kn,0,0]

	#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align
	
    done
done
