#!/bin/bash

process_att(){

    ATT_SO3="$(rospack find att_so3)"

    mkdir -p $2/microstrain/$3/configs
    mkdir -p $2/microstrain/$3/processed
    mkdir -p $2/microstrain/$3/pdfs
    
    MST=$2/microstrain/$3.MST
    CSV=$2/microstrain/$3/processed/$1.csv
    CONFIG=$2/microstrain/$3/configs/$1.m
    PDF=$2/microstrain/$3/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS

    python $ATT_SO3/python/gen_config_file.py -i $MST -o $CSV -c $CONFIG -z $4 -R $5 -a $6

    rosrun att_so3 mems_process $CONFIG

    python $ATT_SO3/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}





EXP=exp1

DIR=/log
LOG=2018_07_18_18_11
HZ=1000
rpy_align=[0,0,0]
rpy_r0=[0,0,0]
k_acc=[0.1,0.1,0.1]
k_mag=[1,1,1]
k_acc_bias=[0.01,0.01,0.1]
k_ang_bias=[0.0001,0.0001,0.01]
k_mag_bias=[1,1,1]


process_att $EXP $DIR $LOG $HZ $rpy_r0 $rpy_align
























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
