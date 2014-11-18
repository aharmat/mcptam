#include <mcptam/Sample.h>
#include <tr1/random>

namespace sample_gen{
std::tr1::ranlux_base_01 gen_real(time(NULL));
std::tr1::mt19937 gen_int;
}

int Sample::uniform(int from, int to)
{
  std::tr1::uniform_int<int> unif(from, to);
  int sam = unif(sample_gen::gen_int);
  return  sam;
}

double Sample::uniform()
{
  std::tr1::uniform_real<double> unif(0.0, 1.0);
  double sam = unif(sample_gen::gen_real);
  return  sam;
}

double Sample::gaussian(double sigma)
{
  std::tr1::normal_distribution<double> gauss(0.0, sigma);
  double sam = gauss(sample_gen::gen_real);
  return  sam;
}

void Sample::setRealSeed(float seed)
{
  sample_gen::gen_real.seed(seed);
}

void Sample::setIntSeed(int seed)
{
  sample_gen::gen_int.seed(seed);
}
