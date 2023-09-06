#ifndef STOP_UTIL_UTIL
#define STOP_UTIL_UTIL


#include <algorithm>
#include <functional>
#include <vector>
#include <cassert>
#include <cmath>
#include <fstream>





// Vector Norm
template <typename T>
double vnorm(const std::vector<T> vec)
{
	double sum = 0.0;
	
	for (auto &&x : vec){
				sum += x*x;
		}
	
	return sqrt(sum);
}


// MOVE ME
template <typename T>
void print_vec(T vec)
{
	std::cout << "Vector: ";
	for (auto const &i: vec) std::cout << i << ' ';
	
	std::cout << std::endl;
}


// MOVE ME
// template <typename T, typename U>
// void print_mat(T mat< U, vec>)
// {
// 	std::cout << "Vector: ";
// 	for (auto const &i: vec) std::cout << i << ' ';
	
// 	std::cout << std::endl;
// }
template <typename T>
void print_mat(T mat)
{
	for (const auto &row : mat) {
		for (const auto &c : row) {
			std::cout << c << " ";
		}
		std::cout << std::endl;
	}
}

// template <typename Time, typename V>
// void print_v2f(std::ofstream &fname, Time t, V vec)
// {
// 	fname << t;
// 	for (auto const &i: vec) fname << " " << i;
// 	fname << "\n";

// }

template <typename Time, typename V>
void print_v2f(std::ofstream &fname, Time t, V vec)
{
	fname << t;
	fname << " " << vec;
	fname << "\n";

}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



#endif //STOP_UTIL_UTIL