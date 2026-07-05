#ifndef WAREHOUSE_UTILS_CSV_LOGGER_HPP_
#define WAREHOUSE_UTILS_CSV_LOGGER_HPP_

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <mutex>

namespace warehouse_utils {

/// Simple thread-safe CSV logger for evaluation metrics.
///
/// Writes header row on first log() call. Each subsequent log()
/// appends one data row. Flushes after every write.
class CsvLogger {
public:
  explicit CsvLogger(const std::string& filepath,
                     const std::vector<std::string>& columns);

  /// Log one row of values (must match column count).
  void log(const std::vector<double>& values);

  /// Close the file.
  void close();

  const std::string& path() const { return filepath_; }

private:
  std::string   filepath_;
  std::ofstream file_;
  std::mutex    mutex_;
  bool          header_written_ = false;
};

// ── Inline implementation ──────────────────────────────────

inline CsvLogger::CsvLogger(const std::string& filepath,
                            const std::vector<std::string>& columns)
  : filepath_(filepath) {
  file_.open(filepath, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) return;

  for (size_t i = 0; i < columns.size(); ++i) {
    if (i > 0) file_ << ',';
    file_ << columns[i];
  }
  file_ << '\n';
  file_.flush();
  header_written_ = true;
}

inline void CsvLogger::log(const std::vector<double>& values) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!file_.is_open()) return;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) file_ << ',';
    file_ << values[i];
  }
  file_ << '\n';
  file_.flush();
}

inline void CsvLogger::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_.is_open()) file_.close();
}

}  // namespace warehouse_utils

#endif
