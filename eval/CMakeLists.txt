# exclude certain files
# note the source dir on each 
set (excluded_evals "")

# Build evals
# Use gtsamAddTestsGlob since evals are just tests, separated due to taking
# much longer to run
gtsamAddTestsGlob(eval "eval*.cpp" "${excluded_evals}" "gtsam")
