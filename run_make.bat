@setlocal

cd _build_make\
make
UnscentedKalmanFilter.exe ../data/sample-laser-radar-measurement-data-2.txt ../data/out-2.txt 

@endlocal