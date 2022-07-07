#include <Eigen/Dense>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Matrix2f Mat2f;

class GRV {
	private:
		Vec2f _mu;
		Mat2f _sigma;
		Mat2f _sigma_inv;
		float _det_sigma;
	public:
		// mu : mean | sigma : variance
		GRV(const Vec2f &__mu, const Mat2f &__sigma) : _mu(__mu), _sigma(__sigma) {
			_simga_inv = _sigma.inverse();
			_det_sigma = _sigma.determinant();	
		};
		Vec2f operator(const Vec2f &__y) {
			Mat2f A = (__y-_mu).transpose()*_sigma_inv*(__y-_mu);
			Vec2f a = A.colwise().exp();
			std::cout<<"a : "<<a<<"\n";
		}
}

class KF {
	private:
	protected:
	public:
		KF() {};
			


}
