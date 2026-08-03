#ifndef BALANCE_BENCHMARK_CSV_WRITER_HPP
#define BALANCE_BENCHMARK_CSV_WRITER_HPP

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string_view>
#include <type_traits>

namespace balance::benchmark {

class CsvWriter {
public:
    CsvWriter(
        const std::filesystem::path &path,
        std::initializer_list<std::string_view> header);

    void begin_row();

    template<typename T>
    CsvWriter &value(const T &value) {
        prepare_value();
        if constexpr (std::is_convertible_v<const T &, std::string_view>) {
            write_text(row_, std::string_view(value));
        } else {
            row_ << value;
        }
        return *this;
    }

    void end_row();
    void flush();

private:
    static void write_text(std::ostream &output, std::string_view value);
    void prepare_value();

    std::ofstream output_;
    std::ostringstream row_;
    std::size_t column_count_{};
    std::size_t current_column_{};
    bool row_active_{};
};

} // namespace balance::benchmark

#endif
