# ORB_SLAM2_aruco

This SLAM system is based on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2.git).

I add fiducial marker--Aruco to deal with scale problem, scale drift, and tracking in large-scale environment.

I learn a lot from [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62), and modified the code from my personal understanding.

<img src="map1.png" alt="map1" style="zoom:33%;" />

<img src="large-scene2.png" alt="large-scene2" style="zoom:33%;" />



### Function:

1. Initializing map by Aruco or Keypoints. If initial by Keypoints, there is a module to correct scale.(In the LocalMapping thread, I commented this code because I usually use aruco to initial map. But it still work)
2. Tracking by MapPoint and MapAruco.
3. LocalMapping thread can update all features and keyframe.
4. Detecting Loop by MapAruco or MapPoints. It also can correct loop by aruco and do global BA.
5. Show Aruco when draw the map.

### Details:

1. Use MapAruco to represent marker features, see in MapAruco.h file.
2. In tracking thread, modified and add:
   -  MonocularInitialization(): include the way initialized by Aruco.
   -  TrackByAruco()
   -  RelocalizationByAruco()
   -  NeedNewKeyFrame(): KeyFrame is needed when there is new Aruco.
   -  CreateNewKeyFrame(): Creating new MapAruco features.
3. In LocalMapping thread, modified and add:
   - CreateArucoMapPoints(): to correct scale
4. In LoopClosing thread, modified and add:
   - DetectLoopByAruco()
   - ComputeSim3ByAruco();
   - CorrectLoopByAruco();
5. Modify a lot in class Optimizer, class Map, class Initializer.
6. Modify a little in class frame, class MapPoint, class MapDrawer, class Keyframe.
7. Add file InitKeyFrame.h, SystemSetting.h to save the map. 
8. Add Aruco in the Thirdparty file.

### Run

```bash
./mono_tum path_to_vocabulary    videofile  cameraparams.yml outposes ARUCO_DIC 
```

like:

```
./Examples/Monocular/mono_cvcam Vocabulary/ORBvoc.txt /home/(user name)/data/spm-slam/video1.mp4 /home/(user name)/data/spm-slam/camera1.yml /home/(user name)/data/spm-slam/save-pose-file-name.txt ARUCO  
```

