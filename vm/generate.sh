make clean
make generate=1
./vm ../scenarios 1001 > bin1.cpp
./vm ../scenarios 2001 > bin2.cpp
./vm ../scenarios 3001 > bin3.cpp
./vm ../scenarios 4001 > bin4.cpp
make clean
make
