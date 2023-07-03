*** Always use git bash for windows


Required tools:

cmake
qt
opencv
cuda


OPENCV INSTALL:


0. Install cuda + cudann

1. download opencv source 4.7.0
https://opencv.org/releases/

2. download opencv_contrib 4.7.0
![Alt text](doc_images/image.png)

CMAKE gui -> copy this, press configure
![Alt text](doc_images/image2.png)

Press configure, then set as follows:
![Alt text](doc_images/image-0.png)

Will take a while

	- Enable cuda
	
![Alt text](doc_images/image-1.png)
![Alt text](doc_images/image-2.png)
![Alt text](doc_images/image-3.png)
![Alt text](doc_images/image-4.png)
Set extra modules to where you installed opencv_contrib
![Alt text](doc_images/image-5.png)
	- hit configure again (verify no errors)
![Alt text](doc_images/image-6.png)
Set install prefix (I put it in C:/opencv/install)
![Alt text](doc_images/image-7.png)
Remove debug in below (should say only release):
![Alt text](doc_images/image-8.png)
Finally hit generate
Open command prompt and run:
![Alt text](doc_images/image-9.png)