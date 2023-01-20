function plotBayesTree(bayesTree)
% plotBayesTree saves as dot file, renders, and shows as image
% Needs dot installed

bayesTree.saveGraph('/tmp/bayesTree.dot')
!dot -Tpng -o /tmp/dotImage.png /tmp/bayesTree.dot
dotImage=imread('/tmp/dotImage.png');
imshow(dotImage)

end