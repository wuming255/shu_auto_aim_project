#ifndef SHU_AUTO_AIM_RUNTIME_CSV_LOGGER_HPP
#define SHU_AUTO_AIM_RUNTIME_CSV_LOGGER_HPP

#include <cstddef>
#include <fstream>
#include <string>

#include "control/auto_aim_runtime.hpp"

namespace srm::app
{

class RuntimeCsvLogger final
{
public:
  // path 为空时禁用文件输出；非空但无法创建时抛 std::runtime_error。
  explicit RuntimeCsvLogger(std::string path = {});

  RuntimeCsvLogger(const RuntimeCsvLogger &) = delete;
  RuntimeCsvLogger & operator=(const RuntimeCsvLogger &) = delete;

  void log(const control::RuntimeStepResult & result);

  bool enabled() const noexcept;
  std::size_t rows_written() const noexcept;
  const std::string & path() const noexcept;

private:
  static std::string csv_field(const std::string & value);
  static std::string join_reasons(const control::SafetyOutput & safety);
  void write_header();

  std::string path_;
  std::ofstream stream_;
  std::size_t rows_written_ = 0;
};

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_RUNTIME_CSV_LOGGER_HPP
