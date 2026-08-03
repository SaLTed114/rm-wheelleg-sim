#include "csv_writer.hpp"

#include <iomanip>
#include <stdexcept>
#include <string>

namespace balance::benchmark {

CsvWriter::CsvWriter(
    const std::filesystem::path &path,
    const std::initializer_list<std::string_view> header
) : column_count_(header.size()) {
    if (column_count_ == 0U) {
        throw std::invalid_argument("CSV header must not be empty");
    }

    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    output_.open(path, std::ios::trunc);
    if (!output_) {
        throw std::runtime_error(
            "failed to open CSV file '" + path.string() + "'");
    }

    std::size_t column = 0U;
    for (const std::string_view name : header) {
        if (column != 0U) output_ << ',';
        write_text(output_, name);
        ++column;
    }
    output_ << '\n';
}

void CsvWriter::begin_row() {
    if (row_active_) throw std::logic_error("CSV row is already active");

    row_.str(std::string{});
    row_.clear();
    row_ << std::setprecision(10);
    current_column_ = 0U;
    row_active_ = true;
}

void CsvWriter::end_row() {
    if (!row_active_) throw std::logic_error("CSV row is not active");
    if (current_column_ != column_count_) {
        row_active_ = false;
        throw std::logic_error(
            "CSV row has " + std::to_string(current_column_) +
            " columns, expected " + std::to_string(column_count_));
    }

    output_ << row_.str() << '\n';
    row_active_ = false;
}

void CsvWriter::flush() {
    output_.flush();
}

void CsvWriter::write_text(
    std::ostream &output, const std::string_view value
) {
    const bool quote = value.find_first_of(",\"\r\n") !=
        std::string_view::npos;
    if (!quote) {
        output << value;
        return;
    }

    output << '"';
    for (const char character : value) {
        if (character == '"') output << '"';
        output << character;
    }
    output << '"';
}

void CsvWriter::prepare_value() {
    if (!row_active_) throw std::logic_error("CSV row is not active");
    if (current_column_ >= column_count_) {
        throw std::logic_error("CSV row has too many columns");
    }
    if (current_column_ != 0U) row_ << ',';
    ++current_column_;
}

} // namespace balance::benchmark
