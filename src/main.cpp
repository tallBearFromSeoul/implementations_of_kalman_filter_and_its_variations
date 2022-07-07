#include "kf.hpp"

int main(int argc, char* argv[]) {
	Vec2f mu {0,0};
	Mat2f sigma {1,0,0,1}; 
	GRV(mu, sigma);



	return 0;
}
