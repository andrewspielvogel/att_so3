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

    python $ATT_SO3/python/gen_config_file.py -i $MST -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_mag $8 --k_acc_bias $9 --k_ang_bias ${10} --k_mag_bias ${11} --acc_bias ${12} --ang_bias ${13} --mag_bias ${14} --k_g ${15} --k_north ${16}

    rosrun att_so3 mems_process $CONFIG

    python $ATT_SO3/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}





EXP=exp8

DIR=/log
LOG=test
HZ=1000
rpy_align=[0,0,3.14]
rpy_ro=[0,0,0.1]
k_acc=[0.1,0.1,0.1]
k_mag=[0.1,0.1,0.1]
k_acc_bias=[0.05,0.05,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000025,0.000025,0.000025]
#k_ang_bias=[0,0,0]
k_mag_bias=[0.5,0.5,1]
#k_mag_bias=[0,0,0]
acc_bias=[0,0,0]
ang_bias=[0,0,0]
mag_bias=[0,0,0]
k_g=[0.25,0.25,0.25]
k_north=[0.1,0.1,0.1]


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
