display("installing Julia packages")
using Pkg
Pkg.add("LinearAlgebra")
Pkg.add("Statistics")
Pkg.add("ReferenceFrameRotations")
Pkg.add("DelimitedFiles")
Pkg.add("Libdl")
Pkg.add("Random")
Pkg.add("PyCall")           # optional
Pkg.add("SpecialFunctions") # optional
Pkg.add("PlotlyJS")         # optional
display("Julia packages installed")

display("compiling C++ libraries")
compile_nanoflann_wrapper = `clang++ -O3 -shared -fPIC -o c_libs/flann_wrap.so c_libs/nanoflann/flann_wrap.cpp`
compile_mpnn2_wrapper 	  = `g++ -O3 -funroll-loops -fPIC -Ic_libs/mpnn2 -shared c_libs/nn_search.cc c_libs/mpnn2/libDNN.a -o c_libs/libnn.so`
compile_RAPID = `make -C c_libs/rapid`
compile_RAPID_wrapper     = `g++ -O3 -funroll-loops -fPIC -Ic_libs/rapid -shared c_libs/wrap.cc c_libs/rapid/libRAPID.a -o c_libs/libwrap.so`

run(compile_RAPID)

run(compile_nanoflann_wrapper)
run(compile_mpnn2_wrapper)
run(compile_RAPID_wrapper)

display("C++ libraries compiled")
