I'm now adding some new features in this program. I will open the whole source when I finish it. Please be waiting.

# ORB_SLAM2_aruco

This SLAM system is based on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2.git).

I added fiducial markers--Aruco to obtain the scale, decrease scale drift, and track in large-scale environment.

I learned a lot from [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62), and wrote the codes from my personal understanding.

<img src="pic4.png" alt="map1"  />

![pic5](pic5.png)



### Function:

1. It can initialize map by Aruco or Keypoints. If initial by Keypoints, there is a module to correct scale.(In the LocalMapping thread, I commented this code because I usually use aruco to initial map. But it still work)
2. It can track by MapPoint and MapAruco features.
3. It can update all features and keyframe in localMapping thread.
4. It can detect and correct loop by MapAruco or MapPoints, and update all features in the global bundle adjustment.
5. It can show Aruco in the map.

## Notice:

The code in `/Examples/Monocular/mono_cvcam.cc` is come from [SPM-SLAM](http://www.uco.es/investiga/grupos/ava/node/58) 

### Citation

R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: a Versatile and Accurate Monocular SLAM System,” IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147–1163, 2015.

Rafael Muñoz-Salinas,Manuel J. Marín-Jiménez Manuel and Rafael Medina-Carnicer , "SPM-SLAM: Simultaneous Localization and Mapping with Squared Planar Markers", September 2018 Pattern Recognition DOI: 10.1016/j.patcog.2018.09.003

Rafael Muñoz-Salinas, R. Medina-Carnicer, "UcoSLAM: Simultaneous localization and mapping by fusion of keypoints and squared planar markers," Pattern Recognition, 2020,107193

