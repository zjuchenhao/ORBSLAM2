#/bin/bash

# ../Examples/Monocular/mono_pinhole \
# ../Vocabulary/ORBvoc.txt \
# ../Examples/Monocular/pinhole.yaml \
# "/home/chenhao/dataset/NewPAL/501GC/PAL-90/Pin/zhukezhen" \
# 80 \
# 1600 \
# 1

# s1: 25~957 960~1625
# s2: 10~140
# s3: 12~342
# s7: 10~360
# s12: 10~314 315~435 483~625 1487~1772

i=2
../Examples/Monocular/mono_pinhole_withGT \
../Vocabulary/ORBvoc.txt \
../Examples/Monocular/pinhole.yaml \
"/home/chenhao/dataset/NewPAL/501GC/PAL-90-T265/Pin/s$i" \
10 \
270
mv KeyFrameTrajectory.txt orbslam2_s$i.txt
