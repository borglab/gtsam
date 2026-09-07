// GNC templates live in gtsam in C++; also expose their CUDA instantiations
// alongside the inner optimizers without registering the native types twice.
m_.attr("cuda").attr("GncSfmLMParams") = m_.attr("GncCudaSfmLMParams");
m_.attr("cuda").attr("GncSfmLMOptimizer") = m_.attr("GncCudaSfmLMOptimizer");
