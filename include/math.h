#ifndef STOP_UTIL_MATH
#define STOP_UTIL_MATH


#include <algorithm>
#include <functional>
#include <vector>
#include <cassert>
#include <fstream>




// Two vectors, element wise operations

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::plus<T>());
    return result;
}


template <typename T>
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::minus<T>());
    return result;
}


template <typename T>
std::vector<T> operator*(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::multiplies<T>());
    return result;
}

template <typename T>
std::vector<T> operator/(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::divides<T>());
    return result;
}


// Vector & scalar, element wise operations


template <typename T>
std::vector<T> operator*(const T& scalar, const std::vector<T>& vec)
{
    std::vector<T> result(vec.size());
    // result.reserve(vec.size());

				   
	std::transform(vec.begin(), vec.end(), result.begin(),
                   [&scalar](double element) { return element *= scalar; });			   
				   
    return result;
}



#endif //STOP_UTIL_MATH