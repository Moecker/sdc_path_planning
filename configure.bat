cmake "-B_build_make" "-H." -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="D:/Udacity/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake "-B_build_vs" "-H." -G "Visual Studio 15 2017 Win64" -DCMAKE_TOOLCHAIN_FILE="D:/Udacity/vcpkg/scripts/buildsystems/vcpkg.cmake"
