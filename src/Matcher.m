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
          
          imds = imageDatastore(imgList,'Labels', imgList);
          bag = bagOfFeatures(imds);
          obj.classifier = trainImageCategoryClassifier(imds, bag);
          %{
          for i=1:length(imgList)
              img = imread(imgList{i});
              img = readGray(obj, img);
              points = detectSURFFeatures(img);
              [featureDesc, validPoints] = extractFeatures(img, points);
              obj.imgDesc{i} = {img, featureDesc, validPoints};
          end
          %}
      end
      
      function matches = imageMatch(obj, image)
          %img = readGray(obj, image);
          
          predict(obj.classifier, image);
          %{
          points = detectSURFFeatures(img);
          [desc, validPoints] = extractFeatures(img, points);
 
          matches = zeros(length(obj.imgList), 1);
          for i=1:length(obj.imgList)
              original = obj.imgDesc{i}{1};
              pDesc = obj.imgDesc{i}{2};
              pValidPoints = obj.imgDesc{i}{3};
              indexPairs = matchFeatures(pDesc, desc);
              
              matchedOriginal  = pValidPoints(indexPairs(:,1));
              matchedDistorted = validPoints(indexPairs(:,2));
              
              [tform, inlierDistorted, inlierOriginal, status] = estimateGeometricTransform(...
                matchedDistorted,...
                matchedOriginal,...
                'similarity'...
              );
              
              if status == 0
                  outputView = imref2d(size(original));
                  recovered  = imwarp(img, tform, 'OutputView', outputView);
                  figure, imshowpair(original,recovered,'montage');
                  
                  z = imabsdiff(original, recovered);
                  figure, imshow(z);
              else
                  disp('Error:')
                  status
              end
          end
          %}
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