#!/bin/bash
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s1/d435 \
        100 \
        1000 \
        $i
    done
done
mkdir -p s1
mv 2019* s1
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s2/d435 \
        200 \
        1000 \
        $i
    done
done
mkdir -p s2
mv 2019* s2
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s2/d435 \
        950 \
        1800 \
        $i
    done
done
mkdir -p s3
mv 2019* s3
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s3/d435 \
        0 \
        1600 \
        $i
    done
done
mkdir -p s4
mv 2019* s4
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s4/d435 \
        500 \
        1100 \
        $i
    done
done
mkdir -p s5
mv 2019* s5
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s5/d435 \
        500 \
        1100 \
        $i
    done
done
mkdir -p s6
mv 2019* s6
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s5/d435 \
        1500 \
        2100 \
        $i
    done
done
mkdir -p s7
mv 2019* s7
#####################################
for i in {1..10}
do
    for j in {1..5}
    do
        ../Examples/Monocular/mono_rs \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/d435.yaml \
        /home/chenhao/dataset/PALVO_Experiment_new/control_group/s5/d435 \
        2300 \
        3400 \
        $i
    done
done
mkdir -p s8
mv 2019* s8
