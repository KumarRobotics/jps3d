#include <yaml-cpp/yaml.h>

template <class Ti, class Tf>
class MapReader {
  public:
    MapReader(const std::string& file) {
      try {
      YAML::Node config = YAML::LoadFile(file);
        const std::vector<double>& origin_vec = config["origin"].as<std::vector<double>>();
        for(int i = 0; i < origin_vec.size(); i++)
          origin_(i) = origin_vec[i];

        const std::vector<int>& dim_vec = config["dim"].as<std::vector<int>>();
        for(int i = 0; i < dim_vec.size(); i++)
          dim_(i) = dim_vec[i];

        data_ = config["dim"].as<std::vector<signed char>>();

        resolution_ = config["resolution"].as<double>();
        exist_ = true;
      } catch (YAML::ParserException& e) {
        //std::cout << e.what() << "\n";
        exist_ = false;
      }
    }

    bool exist() { return exist_; }
    Tf origin() { return origin_; }
    Ti dim() { return dim_; }
    double resolution() { return resolution_; }
    std::vector<signed char> data() { return data_; }
  private:
    Tf origin_;
    Ti dim_;
    double resolution_;
    std::vector<signed char> data_;

    bool exist_ = false;
};
