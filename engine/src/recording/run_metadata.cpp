/**
 * @file
 */

#include <concepts>
#include <format>
#include <fstream>
#include <locale>
#include <span>
#include <sstream>
#include <stdexcept>

#include <picosha2.h>

#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Escape a string as a JSON string literal body.
 *
 * @param value Raw string.
 * @return Escaped string, without the surrounding quotes.
 */
std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());

    for (const unsigned char character : value) {
        switch (character) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                if (character < 0x20) {
                    escaped += std::format("\\u{:04x}", character);
                } else {
                    escaped.push_back(static_cast<char>(character));
                }
        }
    }

    return escaped;
}

/**
 * @brief Append one quoted, escaped string field.
 *
 * @param out Stream being built.
 * @param name Name of the field.
 * @param value Value of the field.
 * @param last Whether this is the last field of the object.
 */
void field(std::ostringstream& out, const char* name, const std::string& value, bool last = false) {
    out << "  \"" << name << "\": \"" << json_escape(value) << '"' << (last ? "\n" : ",\n");
}

/**
 * @brief Append one boolean field, as a JSON literal rather than a string.
 *
 * @param out Stream being built.
 * @param name Name of the field.
 * @param value Value of the field.
 * @param last Whether this is the last field of the object.
 */
void field(std::ostringstream& out, const char* name, bool value, bool last = false) {
    out << "  \"" << name << "\": " << (value ? "true" : "false") << (last ? "\n" : ",\n");
}

/**
 * @brief Append one numeric field, formatted like std::ostream would.
 *
 * @note Constrained to arithmetic types so that a string literal cannot bind
 *       here and be written unquoted and unescaped.
 *
 * @param out Stream being built.
 * @param name Name of the field.
 * @param value Value of the field.
 * @param last Whether this is the last field of the object.
 */
template <typename T>
requires std::is_arithmetic_v<T>
void field(std::ostringstream& out, const char* name, T value, bool last = false) {
    out << "  \"" << name << "\": " << value << (last ? "\n" : ",\n");
}
}  // namespace

std::string RunMetadata::to_json() const {
    std::ostringstream out;

    out.imbue(std::locale::classic());
    out << "{\n";
    field(out, "target", this->target);
    field(out, "firmware_sha", this->firmware_sha);
    field(out, "robot_path", this->robot_path);
    field(out, "robot_sha256", this->robot_sha256);
    field(out, "scenario_path", this->scenario_path);
    field(out, "maze_path", this->maze_path);
    field(out, "model_sha256", this->model_sha256);
    field(out, "mujoco_version", this->mujoco_version);
    field(out, "compiler", this->compiler);
    field(out, "build_type", this->build_type);
    field(out, "args", this->args);
    field(out, "seed", this->seed);
    field(out, "ideal", this->ideal);
    field(out, "loop_time_us", this->loop_time_us);
    field(out, "timestep", this->timestep);
    field(out, "steps_per_tick", this->steps_per_tick);
    field(out, "record_every", this->record_every);
    field(out, "requested_ticks", this->requested_ticks);
    field(out, "ticks", this->ticks);
    field(out, "sim_time", this->sim_time);
    field(out, "stopped_at", this->stopped_at);
    field(out, "final_z", this->final_z);

    for (const MetadataField& target_field : this->target_fields) {
        field(out, target_field.name.c_str(), target_field.value);
    }

    field(out, "warnings_total", this->warnings_total);
    field(out, "serial_dropped_bytes", this->serial_dropped_bytes);
    field(out, "bridge_dropped_frames", this->bridge_dropped_frames);
    field(out, "interactive", this->interactive);
    out << "  \"events\": [";

    for (std::size_t i = 0; i < this->events.size(); i++) {
        const RunEvent& event = this->events[i];
        out << (i > 0 ? "," : "") << "\n    {" << R"("time": )" << event.time << R"(, "kind": ")"
            << json_escape(event.kind) << R"(", "detail": ")" << json_escape(event.detail) << R"("})";
    }

    out << (this->events.empty() ? "]\n" : "\n  ]\n") << "}\n";
    return out.str();
}

void RunMetadata::write(const std::filesystem::path& path) const {
    std::ofstream file(path);

    if (not file.is_open()) {
        throw std::runtime_error("failed to open '" + path.string() + "' for writing");
    }

    file << this->to_json();
}

std::string RunMetadata::sha256_of_text(const std::string& text) {
    return picosha2::hash256_hex_string(text);
}

std::string RunMetadata::sha256_of(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);

    if (not file.is_open()) {
        return {};
    }

    return picosha2::hash256_hex_string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

std::string RunMetadata::join_args(std::span<char*> arguments) {
    std::string joined;

    for (std::size_t i = 1; i < arguments.size(); i++) {
        joined += (i > 1 ? " " : "");
        joined += arguments[i];
    }

    return joined;
}
}  // namespace micras::sim
