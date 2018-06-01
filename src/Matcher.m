classdef Matcher < handle
    
  properties
    imgList
    imgDesc
    
    classifier
  end
  
  methods
      function obj = Matcher(imgList)
          obj.imgList = imgList;          
          obj.imgDesc = {};
     
          for i=1:length(imgList)
              img = imread(imgList{i});
              img = readGray(obj, img);
              points = detectSURFFeatures(img);
              [featureDesc, validPoints] = extractFeatures(img, points);
              obj.imgDesc{i} = {img, featureDesc, validPoints};
          end
      end
      
      function matches = imageMatch(obj, image)
          img = readGray(obj, image);
          
          points = detectSURFFeatures(img);
          [desc, validPoints] = extractFeatures(img, points);
 
          matches = zeros(length(obj.imgList), 1);
          for i=1:length(obj.imgList)
              original = obj.imgDesc{i}{1};
              pDesc = obj.imgDesc{i}{2};
              pValidPoints = obj.imgDesc{i}{3};
              indexPairs = matchFeatures(pDesc, desc, 'MaxRatio', 0.8);
              
              matchedOriginal  = pValidPoints(indexPairs(:,1));
              matchedDistorted = validPoints(indexPairs(:,2));
              
              [tform, inlierDistorted, inlierOriginal, status] = estimateGeometricTransform(...
                matchedDistorted,...
                matchedOriginal,...
                'projective',...
                'MaxNumTrials', 2000,...
                'MaxDistance', 5 ...
              );
          
              if status == 0 && size(inlierDistorted.Location, 1) >= 3
                  distordedPc = pointCloud([inlierDistorted.Location, zeros(inlierDistorted.Count, 1)]);
                  originalPc = pointCloud([inlierOriginal.Location, zeros(inlierOriginal.Count, 1)]);

                  [~, ~, rmse] = pcregrigid(distordedPc, originalPc);
                  matches(i) = rmse;
              else
                  matches(i) = Inf;
              end
          end
      end
      
      function grayImg = readGray(obj, img)
          if ndims(img) == 3
              grayImg = rgb2gray(img);
          else
              grayImg = img;
          end
      end
      
  end
end