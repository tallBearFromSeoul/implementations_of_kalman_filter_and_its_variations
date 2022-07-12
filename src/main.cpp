#include "vehicle.hpp"
#include "kf.hpp"

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Matrix2f Mat2f;

int Obs::MAXID = 0;

int main(int argc, char* argv[]) {
	/*
	Vec2f mu = {0,0};
	Mat2f sigma {{0.01,0},{0,0.01}}; 
	GRV grv = GRV(mu, sigma);

	Vec2f test = {0,0};
	float res = grv(test);
	std::cout<<"prob of : "<<test<<"\n  is : "<<res<<"\n";

	for (int i=0; i<10; i++) {
		VecXf res2 = grv.sample();
		mu(0) += float(i)*0.1;
		mu(1) -= float(i)*0.1;
		grv.set_mu(mu);
		std::cout<<"multivariate Gaussian distribution sample : \n"<<res2<<"\n";
	}
	*/
	float scanner_range = 20.f;
	Li_Radar *scanner = new Li_Radar(scanner_range);
	World *world = new World();
	ObjetFactory *factory = new ObjetFactory();
	float max_bounds[6] {-20.f,20.f,-20.f,20.f,0.2f,0.4f};
	factory->createNObstacles(1, max_bounds, world->obstacles());

	KF *kf_jo = new KF("kalman_joseph");
	KF *kf_kg = new KF("kalman_gain");
	KF *kf_rc = new KF("kalman_recursive");

	RowVec2f p_init {0.f,0.f};
	std::unordered_map<int, Vec4f> *xm;

	for (int i=0; i<30; i++) {
		int j=0;
		std::vector<ObsPtr> obstacles_in_range;
		std::cout<<"iteration #"<<i<<"\n";
		scanner->scan(p_init, world->obstacles(), obstacles_in_range);
		xm = kf_rc->update_recursive(obstacles_in_range, scanner);
		xm = kf_kg->update_kalman_gain(obstacles_in_range, scanner, false);
		xm = kf_jo->update_kalman_gain(obstacles_in_range, scanner, true);
		world->update();
	}
	kf_rc->kf_output.close();
	kf_kg->kf_output.close();
	kf_jo->kf_output.close();
	return 0;
}
