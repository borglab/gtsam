// GNC templates live in gtsam in C++; also expose their CUDA instantiations
// alongside the inner optimizers without registering the native types twice.
m_.attr("cuda").attr("GncSparseLMParams") = m_.attr("GncCudaSparseLMParams");
m_.attr("cuda").attr("GncSparseLMOptimizer") = m_.attr("GncCudaSparseLMOptimizer");
