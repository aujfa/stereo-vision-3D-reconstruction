# Stereo Vision 3D Reconstruction
This is a C++ application for 3D image reconstruction from two images using stereo vision technique. Stereo pair was obtained with two low resolution web cameras.

https://user-images.githubusercontent.com/37661642/148678603-79d6584b-1533-4312-9ae6-3bca1f954066.mp4

# Requirements

- Two webcams or stereo camera
- openCV 3.4.0 (with `contrib` module)
- A chessboard pattern `pattern.png` which you can download from [here](https://github.com/opencv/opencv/blob/master/doc/pattern.png)

# Steps

1. Print the chessboard pattern and paste it on a hard surface or display it on your phone or tablet. The print/image needs to be as straight as possible! 
2. Capture multiple images of the chessboard pattern from the different views (as you can see [here](/boards)) for calibration using  `takeChessboardImages()` function from `camera_calibrator.cpp`.
3. I have included one stereo pair. You need to make your own with the cameras you calibrated. Create a stereo pair using `takeStereoPair()` from  `camera_calibrator.cpp`. 
4. Finally, reconstruct the scene with `ReconstructPointCloud()` from  `camera_calibrator.cpp`. 

\* `blendPictures()` is a helper function to ease the process of aligning the cameras for the stereo pair capture. It is not needed for reconstruction.

\* `opencv_world340d.dll` is compiled binary for Visual Studio 2017 32-bit. If you have some other IDE, you need to compile your own binary from source.
# Resources

- Most of the theory behind the code is explained in [3-D Reconstruction with Vision by Venkatesh Tata](https://towardsdatascience.com/3-d-reconstruction-with-vision-ef0f80cbb299)
- Article about Viz from openCV contrib module: [Viz - New 3D visualization module in the OpenCV library](https://sudonull.com/post/113808-Viz-New-3D-Visualization-Module-in-OpenCV-Library-Intel-Blog)
- Instruction how to make your own binary: [Build opencv 3.4 under windows 10 with contrib library, Git Source ,CMAKE ,Visual Studio 2017](https://funvision.blogspot.com/2018/11/build-opencv-34-under-windows-10-with.html)
- If you are interested in details about matrices for calibration, triangulation, etc. there is a great lecture by Cyrill Stachniss: [Photogrammetry](https://youtube.com/playlist?list=PLgnQpQtFTOGRsi5vzy9PiQpNWHjq-bKN1)

