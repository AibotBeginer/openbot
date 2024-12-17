#ifndef FILTERS__FILTER_BASE_HPP_
#define FILTERS__FILTER_BASE_HPP_

#include <string>
#include <typeinfo>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <glog/logging.h>

namespace filters
{

namespace impl
{

inline std::string normalize_param_prefix(std::string prefix)
{
  if (!prefix.empty() && prefix.back() != '.') {
    prefix += '.';
  }
  return prefix;
}

}  // namespace impl

/**
 * \brief A Base filter class to provide a standard interface for all filters
 */
template<typename T>
class FilterBase
{
public:
  FilterBase() : configured_(false) {}

  virtual ~FilterBase() = default;

  /**
   * \brief Configure the filter with parameters
   * \param param_prefix A prefix for all parameter names
   * \param filter_name The name of the filter
   * \param params A map representing parameters
   */
  bool configure(
    const std::string & param_prefix,
    const std::string & filter_name,
    const std::unordered_map<std::string, std::string> & params)
  {
    if (configured_) {
      LOG(WARNING) << "Filter " << filter_name_ << " already being reconfigured";
    }

    filter_name_ = filter_name;
    param_prefix_ = impl::normalize_param_prefix(param_prefix);
    params_ = params;

    configured_ = configure();
    return configured_;
  }

  virtual bool update(const T & data_in, T & data_out) = 0;

  inline const std::string & getName() { return filter_name_; }

protected:
  virtual bool configure() = 0;

  bool getParam(const std::string & name, std::string & value)
  {
    return getParamImpl(name, value);
  }

  bool getParam(const std::string & name, bool & value)
  {
    std::string str_value;
    if (getParamImpl(name, str_value)) {
      value = (str_value == "true");
      return true;
    }
    return false;
  }

  bool getParam(const std::string & name, double & value)
  {
    return getParamFromString<double>(name, value);
  }

  bool getParam(const std::string & name, int & value)
  {
    return getParamFromString<int>(name, value);
  }

  bool getParam(const std::string & name, std::vector<double> & value)
  {
    return getVectorParamFromString<double>(name, value);
  }

private:
  bool getParamImpl(const std::string & name, std::string & value)
  {
    auto full_name = param_prefix_ + name;
    auto it = params_.find(full_name);
    if (it != params_.end()) {
      value = it->second;
      return true;
    }
    return false;
  }

  template<typename PT>
  bool getParamFromString(const std::string & name, PT & value)
  {
    std::string str_value;
    if (getParamImpl(name, str_value)) {
      try {
        value = static_cast<PT>(std::stod(str_value));
        return true;
      } catch (const std::invalid_argument & e) {
        LOG(ERROR) << "Parameter " << name << " could not be converted to number.";
      }
    }
    return false;
  }

  template<typename PT>
  bool getVectorParamFromString(const std::string & name, std::vector<PT> & value)
  {
    std::string str_value;
    if (getParamImpl(name, str_value)) {
      value.clear();
      std::istringstream stream(str_value);
      std::string element;
      while (std::getline(stream, element, ',')) {
        try {
          value.push_back(static_cast<PT>(std::stod(element)));
        } catch (const std::invalid_argument & e) {
          LOG(ERROR) << "Element " << element << " in " << name << " could not be converted.";
          return false;
        }
      }
      return true;
    }
    return false;
  }

protected:
  std::string filter_name_;
  bool configured_;
  std::string param_prefix_;
  std::unordered_map<std::string, std::string> params_;
};

template<typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase() : number_of_channels_(0) {}

  virtual ~MultiChannelFilterBase() = default;

  bool configure(
    size_t number_of_channels,
    const std::string & param_prefix,
    const std::string & filter_name,
    const std::unordered_map<std::string, std::string> & params)
  {
    number_of_channels_ = number_of_channels;
    return FilterBase<T>::configure(param_prefix, filter_name, params);
  }

  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out) = 0;

protected:
  size_t number_of_channels_;
};

}  // namespace filters

#endif  // FILTERS__FILTER_BASE_HPP_