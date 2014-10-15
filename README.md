simple_lane_tracking
====================


The code is written in C++ on my Macbook Pro. The version of OpenCV on my machine is 2.4.9. So, in general, the recent OpenCV 2.4.x should work. The main file is ‘track.cpp’. 

The program includes two major steps which are lane feature (markers) extraction, lane modeling (two sides). RANSAC is used to fit 2D lines robustly once lane markers are spotted. In each image, the result will be either two sides of the lane of interest are detected or none of them are detected.
