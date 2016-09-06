PATH_TO_SEQUENCE=/Users/lixihua/Downloads/database/rgbd_dataset_frei/rgbd_dataset_freiburg1_desk

# [Testing!!!]06.RGB-D example: TUM Dataset
#python associate.py $PATH_TO_SEQUENCE/rgb.txt $PATH_TO_SEQUENCE/depth.txt > associations.txt
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml "$PATH_TO_SEQUENCE" associations.txt

# [Tested!!!]04. Monocular Examples: TUM Dataset
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml "$PATH_TO_SEQUENCE"
