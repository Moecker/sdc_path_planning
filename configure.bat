cmake "-B_build_make" "-H." -G "Unix Makefiles" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake "-B_build_vs" "-H." -G "Visual Studio 15 2017 Win64" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
