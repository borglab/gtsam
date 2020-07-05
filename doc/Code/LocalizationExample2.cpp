// add unary measurement factors, like GPS, on all three poses
noiseModel::Diagonal::shared_ptr unaryNoise =
 noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

