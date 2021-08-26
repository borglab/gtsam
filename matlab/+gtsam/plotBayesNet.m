function plotBayesNet(bayesNet)
% plotBayesNet saves as dot file, renders, and shows as image
% Needs dot installed

bayesNet.saveGraph('/tmp/bayesNet.dot')
!dot -Tpng -o /tmp/dotImage.png /tmp/bayesNet.dot
dotImage=imread('/tmp/dotImage.png');
imshow(dotImage)

end