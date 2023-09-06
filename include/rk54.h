#ifndef STOP_INTEGRATION_rk54
#define STOP_INTEGRATION_rk54


#include <boost/array.hpp>
#include <Eigen/Dense>

	
	

const Eigen::Matrix<double, 6, 1> rk54_c  {
		0.,
		1./4.,
		3./8.,
		12./13.,
		1.,
		1./2.
};


const Eigen::Matrix<double, 1, 1> rk54_a2  {  
         1./4.
};

const Eigen::Matrix<double, 2, 1> rk54_a3  {
        3./32.,
        9./32.
};

const Eigen::Matrix<double, 3, 1> rk54_a4  {        
        1932./2197.,
        -7200./2197.,
        7296./2197.
};

const Eigen::Matrix<double, 4, 1> rk54_a5  {    
        439./216.,
        -8.,
        3680./513.,
        -845./4104.
};
 
const Eigen::Matrix<double, 5, 1> rk54_a6  { 
		-8./27.,
		2.,
		-3544./2565.,
		1859./4104.,
		-11./40.
};


const Eigen::Matrix<double, 6, 1> rk54_bl  {
		 25./216.,
		 0.,
		 1408./2565.,
		 2197./4104.,
		 -1./5.,
		 0.		
};

const Eigen::Matrix<double, 6, 1> rk54_bu  {
		 16./135.,
		 0.,
		 6656./12825.,
		 28561./56430.,
		-9./50.,
		 2./55.
};
		

#endif //STOP_INTEGRATION_rk54