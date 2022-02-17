# Details

Date : 2022-01-12 02:35:44

Directory /home/fernando/Code/CS_SLAM

Total : 57 files,  481118 codes, 1315 comments, 688 blanks, all 483121 lines

[summary](results.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [CMakeLists.txt](/CMakeLists.txt) | Django txt | 130 | 0 | 32 | 162 |
| [README.md](/README.md) | Markdown | 3 | 0 | 3 | 6 |
| [data/SensorsConfiguration.yaml](/data/SensorsConfiguration.yaml) | YAML | 2 | 0 | 0 | 2 |
| [data/depth_sensor.txt](/data/depth_sensor.txt) | Django txt | 19,554 | 0 | 1 | 19,555 |
| [data/dvl_linkquest.txt](/data/dvl_linkquest.txt) | Django txt | 5,565 | 0 | 1 | 5,566 |
| [data/imu_adis.txt](/data/imu_adis.txt) | Django txt | 19,966 | 0 | 1 | 19,967 |
| [data/imu_xsens_mti.txt](/data/imu_xsens_mti.txt) | Django txt | 19,554 | 0 | 1 | 19,555 |
| [data/odometry.txt](/data/odometry.txt) | Django txt | 21,844 | 0 | 1 | 21,845 |
| [data/sonar_micron.txt](/data/sonar_micron.txt) | Django txt | 45,600 | 0 | 1 | 45,601 |
| [data/sonar_seaking.txt](/data/sonar_seaking.txt) | Django txt | 97,502 | 0 | 1 | 97,503 |
| [data/tf.txt](/data/tf.txt) | Django txt | 249,132 | 0 | 1 | 249,133 |
| [examples/cave.cpp](/examples/cave.cpp) | C++ | 130 | 86 | 21 | 237 |
| [include/MSCKF.h](/include/MSCKF.h) | C++ | 49 | 1 | 14 | 64 |
| [include/Atlas.h](/include/Atlas.h) | C++ | 25 | 0 | 8 | 33 |
| [include/Converter.h](/include/Converter.h) | C++ | 11 | 46 | 14 | 71 |
| [include/Drawer.h](/include/Drawer.h) | C++ | 27 | 1 | 9 | 37 |
| [include/EKF.h](/include/EKF.h) | C++ | 40 | 1 | 11 | 52 |
| [include/FrameDrawer.h](/include/FrameDrawer.h) | C++ | 17 | 0 | 5 | 22 |
| [include/Frames.h](/include/Frames.h) | C++ | 30 | 1 | 9 | 40 |
| [include/KeyFrame.h](/include/KeyFrame.h) | C++ | 33 | 8 | 8 | 49 |
| [include/LocalMap.h](/include/LocalMap.h) | C++ | 25 | 0 | 6 | 31 |
| [include/LocalMapping.h](/include/LocalMapping.h) | C++ | 22 | 0 | 7 | 29 |
| [include/LoopClosing.h](/include/LoopClosing.h) | C++ | 26 | 1 | 5 | 32 |
| [include/MapDrawer.h](/include/MapDrawer.h) | C++ | 18 | 0 | 5 | 23 |
| [include/MapPoint.h](/include/MapPoint.h) | C++ | 16 | 1 | 8 | 25 |
| [include/MeasurementPackage.h](/include/MeasurementPackage.h) | C++ | 27 | 5 | 5 | 37 |
| [include/PoseGraph.h](/include/PoseGraph.h) | C++ | 24 | 4 | 4 | 32 |
| [include/RandomVector.h](/include/RandomVector.h) | C++ | 29 | 146 | 34 | 209 |
| [include/ScanFormer.h](/include/ScanFormer.h) | C++ | 42 | 14 | 18 | 74 |
| [include/SetParameters.h](/include/SetParameters.h) | C++ | 11 | 0 | 6 | 17 |
| [include/SonarDrawer.h](/include/SonarDrawer.h) | C++ | 16 | 0 | 5 | 21 |
| [include/System.h](/include/System.h) | C++ | 60 | 3 | 10 | 73 |
| [include/Utils.h](/include/Utils.h) | C++ | 21 | 1 | 6 | 28 |
| [include/Viewer.h](/include/Viewer.h) | C++ | 34 | 3 | 13 | 50 |
| [plot.py](/plot.py) | Python | 26 | 12 | 7 | 45 |
| [poses.txt](/poses.txt) | Django txt | 226 | 0 | 1 | 227 |
| [src/MSCKF.cpp](/src/MSCKF.cpp) | C++ | 81 | 60 | 28 | 169 |
| [src/Atlas.cpp](/src/Atlas.cpp) | C++ | 18 | 0 | 7 | 25 |
| [src/Converter.cpp](/src/Converter.cpp) | C++ | 3 | 7 | 6 | 16 |
| [src/Drawer.cpp](/src/Drawer.cpp) | C++ | 89 | 409 | 102 | 600 |
| [src/EKF.cpp](/src/EKF.cpp) | C++ | 90 | 48 | 14 | 152 |
| [src/Frames.cpp](/src/Frames.cpp) | C++ | 77 | 4 | 16 | 97 |
| [src/KeyFrame.cpp](/src/KeyFrame.cpp) | C++ | 59 | 2 | 21 | 82 |
| [src/LocalMap.cpp](/src/LocalMap.cpp) | C++ | 18 | 0 | 7 | 25 |
| [src/LoopClosing.cpp](/src/LoopClosing.cpp) | C++ | 115 | 69 | 14 | 198 |
| [src/MapPoint.cpp](/src/MapPoint.cpp) | C++ | 9 | 0 | 5 | 14 |
| [src/MeasurementPackage.cpp](/src/MeasurementPackage.cpp) | C++ | 8 | 0 | 8 | 16 |
| [src/RandomVector.cpp](/src/RandomVector.cpp) | C++ | 82 | 45 | 16 | 143 |
| [src/ScanFormer.cpp](/src/ScanFormer.cpp) | C++ | 128 | 162 | 27 | 317 |
| [src/System.cpp](/src/System.cpp) | C++ | 149 | 79 | 35 | 263 |
| [src/Utils.cpp](/src/Utils.cpp) | C++ | 33 | 14 | 6 | 53 |
| [src/Viewer.cpp](/src/Viewer.cpp) | C++ | 95 | 21 | 24 | 140 |
| [src/draw.cpp](/src/draw.cpp) | C++ | 46 | 3 | 4 | 53 |
| [test/TEST_MSCKF.cpp](/test/TEST_MSCKF.cpp) | C++ | 18 | 12 | 13 | 43 |
| [test/TEST_DRAWER.cpp](/test/TEST_DRAWER.cpp) | C++ | 104 | 34 | 30 | 168 |
| [test/TEST_SCANMATCHING.cpp](/test/TEST_SCANMATCHING.cpp) | C++ | 43 | 12 | 17 | 72 |
| [基本的架构.md](/基本的架构.md) | Markdown | 16 | 0 | 6 | 22 |

[summary](results.md)