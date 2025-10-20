#pragma once

#include <adptsysc/object.hh>
#include <adptsysc/integers.hh>
#include <stdint.h>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>

namespace adptsysc {

template <typename T>
class Memory :  public ObjectWithMutableHyperparams, 
                public sc_core::sc_module {
 public:
  tlm_utils::simple_target_socket<Memory> targ_socket;

  sc_core::sc_in<bool> clk;
  sc_core::sc_in<bool> reset;
  sc_core::sc_in<bool> ram_en;  
  sc_core::sc_in<bool> write_en;     
  sc_core::sc_in<bool> in_valid;     
  sc_core::sc_out<bool> out_ready; 

  // Address and data buses
  sc_core::sc_in<std::size_t> addr;
  sc_core::sc_in<T> in_data;
  sc_core::sc_out<T> out_data;

  Memory(sc_core::sc_module_name name, 
        std::size_t size = 0, T* initptr = nullptr)
      : sc_module(name), targ_socket("targ_socket"), 
        mem_size(size), is_init(false) {
    targ_socket.register_b_transport(this, &Memory::b_transport);
    targ_socket.register_get_direct_mem_ptr(this, &Memory::get_direct_mem_ptr);
    targ_socket.register_transport_dbg(this, &Memory::transport_dbg);

    SC_METHOD(memory_read);
    sensitive << clk.pos();
    dont_initialize();

    SC_METHOD(memory_write);
    sensitive << clk.pos();
    dont_initialize();

    if (size > 0) {
      allocate(size);
      if (initptr) {
        std::copy(initptr, initptr + size, mem_data.get());
      }
    }
  }

  virtual ~Memory() = default;

  // Object interface
  json hyperparams() const override {
    return {
      {"otype", "Memory"}, 
      {"size", mem_size},
      {"data_type", typeid(T).name()}
    };
  }

  // Mutable configuration
  void update_hyperparams(const json& params) override {
    if (params.contains("size")) {
      allocate(params["size"].get<std::size_t>());
    }
  }

  // Memory operations
  void allocate(const std::size_t size) {
    if (size == 0) {
      SC_REPORT_ERROR(name(), "Cannot allocate memory of size 0");
      return;
    }

    if (is_init && size == mem_size)
      return;

    mem_data = std::make_unique<T[]>(size);
    mem_size = size;
    is_init = true;
  }

  void allocate(const std::shared_ptr<ParametricObject<T>>& target) {
    allocate(static_cast<std::size_t>(target->n_params()));
  }

  T* data() const { return mem_data.get(); }

  std::size_t size() const { return mem_size; }

  void mem_reset() {
    if (is_init) {
      std::fill(mem_data.get(), mem_data.get() + mem_size, T{});
    }
  }

  bool is_valid_addr(const std::size_t addr) const { 
    return addr < mem_size; 
  }

  // Serialization
  json serialize() const {
    return {
      {"size", mem_size},
      {"data", std::vector<T>(mem_data.get(), mem_data.get() + mem_size)}
    };
  }

  void deserialize(const json& data) {
    if (data.contains("size") && data.contains("data")) {
      auto new_size = data["size"].get<std::size_t>();
      auto vec = data["data"].get<std::vector<T>>();

      if (new_size != vec.size()) {
        throw std::runtime_error("Size mismatch in deserialization");
      }

      allocate(new_size);
      std::copy(vec.begin(), vec.end(), mem_data.get());
    }
  }

  void load_from_file(const fs::path& path, int offset = 0) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs)
      throw std::runtime_error("Cannot open file: " + path.string());

    ifs.seekg(offset);
    ifs.read(reinterpret_cast<char*>(mem_data.get()), mem_size * sizeof(T));
  }

  void save_to_file(const fs::path& path, bool force_binary = false) {
    if (force_binary || !std::is_integral_v<T>) {
      // Binary mode (original behavior)
      std::ofstream ofs(path, std::ios::binary);
      if (!ofs)
        throw std::runtime_error("Cannot open file: " + path.string());
      ofs.write(reinterpret_cast<const char*>(mem_data.get()), mem_size * sizeof(T));
    } else {
      // Hex text mode
      std::ofstream ofs(path);
      if (!ofs)
        throw std::runtime_error("Cannot open file: " + path.string());

      ofs << std::hex << std::setfill('0');
      for (std::size_t i = 0; i < mem_size; ++i) {
        if constexpr (sizeof(T) > 1) {
          ofs << std::setw(sizeof(T)*2);
        }
        ofs << static_cast<uint64_t>(mem_data[i]) << '\n';
      }
    }
  }

 protected:
  void b_transport(tlm::tlm_generic_payload& trans, 
                   sc_core::sc_time& delay) {
    auto cmd = trans.get_command();
    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    std::size_t len = static_cast<std::size_t>(trans.get_data_length());
    u8* ptr = trans.get_data_ptr();

    if (addr + len > mem_size * sizeof(T) || addr % sizeof(T) != 0 || len % sizeof(T) != 0) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return;
    }

    T* base = reinterpret_cast<T*>(reinterpret_cast<u8*>(mem_data.get()) + addr);

    if (cmd == tlm::TLM_READ_COMMAND) {
      std::memcpy(ptr, base, len);
    } else if (cmd == tlm::TLM_WRITE_COMMAND) {
      std::memcpy(base, ptr, len);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }

  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, 
                          tlm::tlm_dmi& dmi_data) {
    // Validate address alignment
    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    if (addr % sizeof(T) != 0) {
        SC_REPORT_ERROR(name(), "DMI request with unaligned address");
        return false;
    }

    // Check address range
    if (!is_valid_addr(addr)) {
      std::ostringstream msg;
      msg << "DMI request out of range (addr=0x" << std::hex << addr 
          << ", max=0x" << (mem_size * sizeof(T) - 1) << ")";
      SC_REPORT_WARNING(name(), msg.str().c_str());
      return false;
    }

    // Configure DMI region
    dmi_data.set_dmi_ptr(reinterpret_cast<u8*>(mem_data.get()));
    dmi_data.set_start_address(0);
    dmi_data.set_end_address(mem_size * sizeof(T) - 1);
    
    // A target wishing to deny read and write access to the DMI 
    // region should set the granted access type
    // to DMI_ACCESS_READ_WRITE, not to DMI_ACCESS_NONE
    dmi_data.allow_read_write();
    
    // Configure timing (example values - adjust as needed)
    dmi_data.set_read_latency(sc_core::sc_time(10, sc_core::SC_NS));
    dmi_data.set_write_latency(sc_core::sc_time(10, sc_core::SC_NS));

    return true;
  }

  unsigned int transport_dbg(tlm::tlm_generic_payload& trans) {
    // Validate command type
    tlm::tlm_command cmd = trans.get_command();
    if (cmd != tlm::TLM_READ_COMMAND && cmd != tlm::TLM_WRITE_COMMAND) {
      trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
      return 0;
    }

    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    u8* ptr = trans.get_data_ptr();
    std::size_t len = trans.get_data_length();
    
    if (addr >= mem_size * sizeof(T)) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return 0;
    }

    // Calculate safe transfer length
    std::size_t remains = static_cast<std::size_t>(mem_size * sizeof(T) - addr);
    std::size_t nbytes = (len < remains) ? len : remains;

    if (nbytes == 0) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return 0;
    }

    if (cmd == tlm::TLM_READ_COMMAND) {
      std::memcpy(ptr, reinterpret_cast<u8*>(mem_data.get()) + addr, nbytes);
    } 
    else { // TLM_WRITE_COMMAND
      std::memcpy(reinterpret_cast<u8*>(mem_data.get()) + addr, ptr, nbytes);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
    return nbytes;
  }

  void memory_read() {
    if (reset.read()) {
      out_ready.write(false);
      return;
    }

    if (ram_en.read() && !write_en.read() && in_valid.read()) {
      std::size_t rdaddr = addr.read();
      if (is_valid_addr(rdaddr)) {
        out_data.write(mem_data[rdaddr]);
        out_ready.write(true);
      } else {
        out_ready.write(false);
        SC_REPORT_WARNING(name(), "Read addr out of bounds");
      }
    } else {
      out_ready.write(false);
    }
  }

  void memory_write() {
    if (reset.read())
      return;

    if (ram_en.read() && write_en.read() && in_valid.read()) {
      std::size_t wraddr = addr.read();
      if (is_valid_addr(wraddr)) {
        mem_data[wraddr] = in_data.read();
      } else {
        SC_REPORT_WARNING(name(), "Write addr out of bounds");
      }
    }
  }

 private:
  std::unique_ptr<T[]> mem_data;
  std::size_t mem_size;
  bool is_init;
};

}  // namespace adptsysc
