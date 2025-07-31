#pragma once

#include <adptsysc/object.hh>

#include <stdint.h>
#include <systemc>

namespace adptsysc {

using json = nlohmann::json;

template <typename T>
class Memory : public ObjectWithMutableHyperparams, public sc_module {
 public:
  tlm_utils::simple_target_socket<Memory> targ_socket;

  sc_in<bool> clk;
  sc_in<bool> reset;
  sc_in<bool> ram_en;  
  sc_in<bool> write_en;     
  sc_in<bool> valid_in;     
  sc_out<bool> ready_out; 

  // Address and data buses
  sc_in<uint32_t> addr;
  sc_in<T> data_in;
  sc_out<T> data_out;

  SC_HAS_PROCESS(Memory);

  Memory(sc_module_name name, uint32_t size = 0, T* init_data = nullptr)
      : sc_module(name), targ_socket("targ_socket"), mem_size(size), mem_init(false) {
    targ_socket.register_b_transport(this, &Memory::b_transport);
    targ_socket.register_get_direct_mem_ptr(this, &Memory::get_direct_mem_ptr);
    targ_socket.register_transport_dbg(this, &Memory::transport_dbg);

    SC_METHOD(read_process);
    sensitive << clk.pos();
    dont_initialize();

    SC_METHOD(write_process);
    sensitive << clk.pos();
    dont_initialize();

    if (size > 0) {
      allocate(size);
      if (init_data) {
        std::copy(init_data, init_data + size, mem_data.get());
      }
    }
  }

  virtual ~Memory() = default;

  // Object interface
  json hyperparams() const override {
    return {{"otype", "Memory"}, {"size", mem_size},
        {"data_type", typeid(T).name()}};
  }

  // Mutable configuration
  void update_hyperparams(const json& params) override {
    if (params.contains("size")) {
      allocate(params["size"].get<uint32_t>());
    }
  }

  // Memory operations
  void allocate(uint32_t size) override {
    if (size == 0) {
      SC_REPORT_ERROR(name(), "Cannot allocate memory of size 0");
      return;
    }

    if (mem_init && size == mem_size)
      return;

    mem_data = std::make_unique<T[]>(size);
    mem_size = size;
    mem_init = true;
  }

  void allocate(const std::shared_ptr<ParametricObject<T>>& target) override {
    allocate(static_cast<uint32_t>(target->n_params()));
  }

  T* data() const { return mem_data.get(); }

  size_t size() const { return mem_size; }

  void reset_memory() {
    if (mem_init) {
      std::fill(mem_data.get(), mem_data.get() + mem_size, T{});
    }
  }

  bool is_valid_address(uint32_t addr) const { return addr < mem_size; }

  // Serialization
  json serialize() const override {
    return {{"size", mem_size},
        {"data", std::vector<T>(mem_data.get(), mem_data.get() + mem_size)}};
  }

  void deserialize(const json& data) override {
    if (data.contains("size") && data.contains("data")) {
      auto new_size = data["size"].get<uint32_t>();
      auto data_vec = data["data"].get<std::vector<T>>();

      if (new_size != data_vec.size()) {
        throw std::runtime_error("Size mismatch in deserialization");
      }

      allocate(new_size);
      std::copy(data_vec.begin(), data_vec.end(), mem_data.get());
    }
  }

  // File I/O
  void load_from_file(
      const std::filesystem::path& path, int offset = 0) override {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs)
      throw std::runtime_error("Cannot open file: " + path.string());

    ifs.seekg(offset);
    ifs.read(reinterpret_cast<char*>(mem_data.get()), mem_size * sizeof(T));
  }

  void save_to_file(const std::filesystem::path& path) override {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs)
      throw std::runtime_error("Cannot open file: " + path.string());

    ofs.write(
        reinterpret_cast<const char*>(mem_data.get()), mem_size * sizeof(T));
  }

 protected:
  // TLM methods
  void b_transport(tlm_generic_payload& trans, sc_time& delay) override {
    auto cmd = trans.get_command();
    uint64_t addr = trans.get_address();
    unsigned int len = trans.get_data_length();
    unsigned char* ptr = trans.get_data_ptr();

    if (addr + len > mem_size * sizeof(T) || addr % sizeof(T) != 0 || len % sizeof(T) != 0) {
      trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
      return;
    }

    T* base = reinterpret_cast<T*>(
        reinterpret_cast<uint8_t*>(mem_data.get()) + addr);

    if (cmd == TLM_READ_COMMAND) {
      std::memcpy(ptr, base, len);
    } else if (cmd == TLM_WRITE_COMMAND) {
      std::memcpy(base, ptr, len);
    }

    trans.set_response_status(TLM_OK_RESPONSE);
  }

  bool 
  get_direct_mem_ptr(tlm_generic_payload& trans, tlm_dmi& dmi_data) override {
    dmi_data.set_dmi_ptr(reinterpret_cast<unsigned char*>(mem_data.get()));
    dmi_data.set_start_address(0);
    dmi_data.set_end_address(mem_size * sizeof(T) - 1);
    dmi_data.set_read_latency(sc_time(10, SC_NS));
    dmi_data.set_write_latency(sc_time(10, SC_NS));
    dmi_data.allow_read_write();
    return true;
  }

  unsigned int transport_dbg(tlm_generic_payload& trans) override {
    uint64_t addr = trans.get_address();
    unsigned int len = trans.get_data_length();
    unsigned char* ptr = trans.get_data_ptr();

    if (addr + len > mem_size * sizeof(T))
      return 0;

    std::memcpy(
        ptr, reinterpret_cast<unsigned char*>(mem_data.get()) + addr, len);
    return len;
  }

  // RTL-style processes
  void read_process() {
    if (reset.read()) {
      ready_out.write(false);
      return;
    }

    if (ram_en.read() && !write_en.read() && valid_in.read()) {
      uint32_t addr = addr.read();
      if (is_valid_address(addr)) {
        data_out.write(mem_data[addr]);
        ready_out.write(true);
      } else {
        ready_out.write(false);
        SC_REPORT_WARNING(name(), "Read addr out of bounds");
      }
    } else {
      ready_out.write(false);
    }
  }

  void write_process() {
    if (reset.read())
      return;

    if (ram_en.read() && write_en.read() && valid_in.read()) {
      uint32_t addr = addr.read();
      if (is_valid_address(addr)) {
        mem_data[addr] = data_in.read();
      } else {
        SC_REPORT_WARNING(name(), "Write addr out of bounds");
      }
    }
  }

 private:
  std::unique_ptr<T[]> mem_data;
  uint32_t mem_size;
  bool mem_init;
};

}  // namespace adptsysc
