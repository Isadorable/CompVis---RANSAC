# CompVis---RANSAC

In this project, I implemented a RANSAC matcher to get rid of false positive points that may occur after running a feature matching algorithm. In this case, the assumption is that when N images are taken at the same time from slightly different positions, the features must be around the same areas.

A first set of matching point is detected by using Matlab built-in SURF, which produces almost certainly false positives. My RANSAC algorithm is then iterated several times on this set. For each random N points in the set (hypothetical inliers), I build a model to fit these inliers, until I find one the fits my whole data best. Since I am using a set of points extracted from N pictures of the same planar surface, I used a Homography Matrice as my model.

Once the best model/Homography matrix H has been found, it is used to discard the points where the distance between point2 and the point1*H is bigger than a fix threshold.

BEFORE (red arrows = wrong matched features):
![Alt text](Before.png?raw=true "Title")
AFTER:
![Alt text](After.png?raw=true "Title")
