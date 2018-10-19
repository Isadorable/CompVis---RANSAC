function bestMap = mymatchRANSAC(im1, im2, threshold)
close all;
    %Parameters
    %iter: number of iterations for the RANSAC algorithm
    %threshold:
    %maxPoints: max number of random points used to compute the homography
    %bestMap: best
    iter = 2000;
    maxPoints = 4;
    bestDistance = inf;
    bestAccepted = 0;



    im1 = rgb2gray(im2double(imread(im1)));
    im2 = rgb2gray(im2double(imread(im2)));

    %use the detectorSURF built in matlab's function to detect the features and extract them
    pointsIm1 = detectSURFFeatures(im1);
    pointsIm2 = detectSURFFeatures(im2);

    [featuresIm1, pointsIm1] = extractFeatures(im1, pointsIm1);
    [featuresIm2, pointsIm2] = extractFeatures(im2, pointsIm2);

    %every feature points is matched between the two images
    indexPairs = matchFeatures(featuresIm1,featuresIm2);
    matchedPoints1 = pointsIm1(indexPairs(:, 1),:);
    matchedPoints2 = pointsIm2(indexPairs(:, 2),:);

    %store the ammount of matched points
    numMatch = int32(size(indexPairs, 1));

    %coordinates of matched features
    mp1Loc =  matchedPoints1.Location;
    mp2Loc =  matchedPoints2.Location;
    %creating homogeneous coordinates
    mp1Loc = [mp1Loc, ones(numMatch, 1)];
    mp2Loc = [mp2Loc, ones(numMatch, 1)];

    figure; showMatchedFeatures(im1, im2, matchedPoints1, matchedPoints2, 'montage');


    for i = 1:iter
        %chose 4 random points from the list for im1 and im2
        randI = randi([1 numMatch],1,maxPoints);
        for rI=1:maxPoints
            rPointsIm1(rI,:) = mp1Loc(randI(rI),:);
            rPointsIm2(rI,:) = mp2Loc(randI(rI),:);
        end

        %compute homograpy
        M = createHomography(rPointsIm1, rPointsIm2);
        %use Singular value decomposition : svd(X) produces a diagonal matrix S of the same dimension as X,
        %with nonnegative diagonal elements in decreasing order, and unitary matrices U and V so that X = U*S*V'.
        [U, S, V] = svd(M);
        h = V(:,9);
        %reshape the result to be a 3x3 matrix
        A = reshape(h,3,3)';

        %Now we have a mapping we have to measure its quality.
        %For this we apply the mapping to all features in the image A and find which feature in image B it ends up closest to
        totIterationDistance = 0;
        accepted = 0;
        coordsI= zeros (numMatch,1);
        for n=1:numMatch
            mappedIm1 = A*mp1Loc(n, :)';
            mappedIm1 = mappedIm1 / mappedIm1(3);
            distance = norm(mp2Loc(n, :) - mappedIm1');
            if (distance < threshold)
                accepted = accepted+1;
                totIterationDistance = totIterationDistance + distance;
                coordsI(accepted) = n;
            end
        end

        if (bestDistance > totIterationDistance || bestAccepted < accepted)
            bestMap = A;
            bestDistance = totIterationDistance;
            bestAccepted = accepted;
            bestCoordsI = coordsI;
            fprintf('\n\nThe best distance is equal to %d\nThe number of points is equal to %d over %d points\n', bestDistance, bestAccepted, numMatch);
        end

    end

    for j=1:bestAccepted
        newCoords(j,:) = indexPairs(bestCoordsI(j),:);
    end
    matchedPoints1 = pointsIm1(newCoords(:, 1),:);
    matchedPoints2 = pointsIm2(newCoords(:, 2),:);
    figure; showMatchedFeatures(im1, im2, matchedPoints1, matchedPoints2, 'montage');

%An homography is a map (transform) that allows one image to be translated, scaled, rotated, skewed, and subject
%to perspective-like distortions
    function result = createHomography(m1,m2)
        m = size(m1,1)*2;
        result = zeros(m,9);
        p = 1;
        d = 1;
        for index=1:m
            if(mod(index,2)~=0)
                result(index,1:3) = -m1(d,1:3);
                result(index,7:9) = m1(d,1:3).*m2(d,1);
                d = d+1;
            else
                result(index,4:6) = -m1(p,1:3);
                result(index,7:9) = m1(p,1:3).*m2(p,2);
                p = p+1;
            end
        end
    end
end