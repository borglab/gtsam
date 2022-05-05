// add unary measurement factors, like GPS, on all three poses
auto unaryNoise =
 noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);

