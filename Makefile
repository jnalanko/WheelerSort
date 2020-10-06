.PHONY: main

main:
	$(CXX) -std=c++17 -g -Wall -Wextra -Wno-sign-compare main.cpp -o wheelersort -O3
