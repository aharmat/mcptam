
class Sample
{
public:
  static int uniform(int from, int to);

  static double uniform();

  static double gaussian(double sigma);
  
  static void setRealSeed(float seed);
  
  static void setIntSeed(int seed);
};
