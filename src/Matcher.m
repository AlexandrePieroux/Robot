classdef Matcher < handle
  properties
    imgList
    imgDesc
    cornerThreshold
  end
  methods
      function obj = Matcher(imgList, cornerThreshold)
          obj.cornerThreshold = cornerThreshold;
          obj.imgList = imgList;          
          obj.imgDesc = {};
          for i=1:length(imgList)
              img = readGray(obj, imgList{i});
              points = detectSURFFeatures(img);
              [obj.imgDesc{i}, ~] = extractFeatures(img, points);
          end
      end
      
      function matches = imageMatch(obj, image)
          img = readGray(obj, image);
          points = detectSURFFeatures(img);
          [desc, ~] = extractFeatures(img, points);
 
          for i=1:length(obj.imgList)
              [nbDesc, ~] = size(obj.imgDesc{i});
              matches(i) = match(obj, desc, obj.imgDesc{i}) / nbDesc;
          end
      end
      
      function num = match(obj, des1, des2)
        distRatio = 0.6;   

        % For each descriptor in the first image, select its match to second image.
        des2t = des2';                          % Precompute matrix transpose
        for i = 1 : size(des1,1)
            dotprods = des1(i,:) * des2t;        % Computes vector of dot products
            [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results

            % Check if nearest neighbor has angle less than distRatio times 2nd.
            if (vals(1) < distRatio * vals(2))
                match(i) = indx(1);
            else
                match(i) = 0;
            end
        end
        num = sum(match > 0);
      end
      
      function grayImg = readGray(obj, img)
          grayImg = imread(img);
          if ndims(grayImg) == 3
                grayImg = rgb2gray(grayImg);
          end
      end
      
      function validCorners = cornerDetection(obj, map)
          corners = detectHarrisFeatures(map);
          [~, corners] = extractFeatures(map, corners);
          corners = corners.Location;
          
          % Agregate the corners based on the Euclidian distance, according to a threshold
          for i = 1 : size(corners, 1)
              distances = pdist2(corners, corners(i,:));
              indexes = find(distances < obj.cornerThreshold);
              validCorners(i,:) = mean(corners(indexes,:), 1);
              corners(indexes,:) = 0;
          end
          validCorners(all(validCorners == 0,2),:) = [];
          
      end
  end
end