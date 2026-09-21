/**
 * @file
 *
 * @brief Storage proxy keeping the serialized maze in memory, blank every run.
 */

#include <array>
#include <cstring>

#include "micras/proxy/storage.hpp"

namespace micras::proxy {
std::unordered_map<uint16_t, std::vector<uint8_t>>& Storage::pages() {
    static std::unordered_map<uint16_t, std::vector<uint8_t>> storage_pages;
    return storage_pages;
}

Storage::Storage(const Config& config) : start_page{config.start_page}, number_of_pages{config.number_of_pages} {
    const auto page = Storage::pages().find(this->start_page);

    if (page == Storage::pages().end() or page->second.size() < sizeof(uint64_t)) {
        return;
    }

    uint64_t header{};
    std::memcpy(&header, page->second.data(), sizeof(header));

    if (header >> 48 != start_symbol) {
        return;
    }

    const auto total_size = static_cast<uint16_t>(header >> 32);
    const auto num_primitives = static_cast<uint16_t>(header >> 16);
    const auto num_serializables = static_cast<uint16_t>(header);

    this->buffer.assign(page->second.begin() + sizeof(header), page->second.begin() + sizeof(header) + 8L * total_size);

    this->primitives = deserialize_var_map<PrimitiveVariable>(this->buffer, num_primitives);
    this->serializables = deserialize_var_map<SerializableVariable>(this->buffer, num_serializables);
}

void Storage::create(const std::string& name, const core::ISerializable& data) {
    this->serializables[name].ram_pointer = &data;
}

void Storage::sync(const std::string& name, core::ISerializable& data) {
    if (this->serializables.contains(name) and this->serializables.at(name).ram_pointer == nullptr) {
        const auto& serializable = this->serializables.at(name);
        data.deserialize(&this->buffer.at(serializable.buffer_address), serializable.size);
    }

    this->create(name, data);
}

void Storage::save() {
    this->buffer.clear();

    for (auto& [name, variable] : this->primitives) {
        if (variable.ram_pointer == nullptr) {
            continue;
        }

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        const auto* aux = reinterpret_cast<const uint8_t*>(variable.ram_pointer);
        variable.buffer_address = this->buffer.size();
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): raw bytes of a primitive variable.
        this->buffer.insert(this->buffer.end(), aux, aux + variable.size);
    }

    for (auto& [name, variable] : this->serializables) {
        if (variable.ram_pointer == nullptr) {
            continue;
        }

        const std::vector<uint8_t> aux = variable.ram_pointer->serialize();
        variable.buffer_address = this->buffer.size();
        variable.size = aux.size();
        this->buffer.insert(this->buffer.end(), aux.begin(), aux.end());
    }

    const auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    this->buffer.insert(this->buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    const auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    this->buffer.insert(this->buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    this->buffer.insert(this->buffer.end(), (8 - (this->buffer.size() % 8)) % 8, 0);
    const auto total_size = static_cast<uint16_t>(this->buffer.size() / 8);

    std::array<uint8_t, 8> header{};

    header[0] = this->serializables.size();
    header[1] = this->serializables.size() >> 8;
    header[2] = this->primitives.size();
    header[3] = this->primitives.size() >> 8;

    header[4] = total_size;
    header[5] = total_size >> 8;
    header[6] = start_symbol & 0xFF;
    header[7] = start_symbol >> 8;

    std::vector<uint8_t> page(header.begin(), header.end());
    page.insert(page.end(), this->buffer.begin(), this->buffer.end());
    Storage::pages()[this->start_page] = std::move(page);
}

template <typename T>
std::vector<uint8_t> Storage::serialize_var_map(const std::unordered_map<std::string, T>& variables) {
    std::vector<uint8_t> buffer;

    for (const auto& [name, variable] : variables) {
        buffer.emplace_back(name.size());
        buffer.insert(buffer.end(), name.begin(), name.end());

        buffer.emplace_back(variable.buffer_address);
        buffer.emplace_back(variable.buffer_address >> 8);

        buffer.emplace_back(variable.size);
        buffer.emplace_back(variable.size >> 8);
    }

    return buffer;
}

template <typename T>
std::unordered_map<std::string, T> Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars) {
    std::unordered_map<std::string, T> variables;

    uint16_t current_addr = 0;

    for (uint16_t decoded_vars = 0; decoded_vars < num_vars; decoded_vars++) {
        const uint8_t var_name_len = buffer.at(current_addr);

        const std::string var_name(buffer.begin() + current_addr + 1, buffer.begin() + current_addr + 1 + var_name_len);
        current_addr += var_name_len + 1;

        variables[var_name].buffer_address = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;

        variables.at(var_name).size = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;
    }

    buffer.erase(buffer.begin(), buffer.begin() + current_addr);
    return variables;
}
}  // namespace micras::proxy
