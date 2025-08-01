#pragma once

#include <adptsysc/object.hh>
#include <adptsysc/integers.hh>
#include <stdint.h>
#include <systemc>

namespace adptsysc {

template <typename T>
class Memory : public ObjectWithMutableHyperparams, public sc_core::sc_module {
 public:
  tlm_utils::simple_target_socket<Memory> targ_socket;

  sc_core::sc_in<bool> clk;
  sc_core::sc_in<bool> reset;
  sc_core::sc_in<bool> ramen;  
  sc_core::sc_in<bool> writeen;     
  sc_core::sc_in<bool> invalid;     
  sc_core::sc_core<bool> outready; 

  // Address and data buses
  sc_core::sc_in<std::size_t> addr;
  sc_core::sc_in<T> indata;
  sc_core::sc_core<T> outdata;

  sc_core::SC_HAS_PROCESS(Memory);

  Memory(sc_core::sc_module_name name, std::size_t size = 0, T* initptr = nullptr)
      : sc_module(name), targ_socket("targ_socket"), memsize(size), isinit(false) {
    targ_socket.register_b_transport(this, &Memory::b_transport);
    targ_socket.register_get_direct_mem_ptr(this, &Memory::get_direct_mem_ptr);
    targ_socket.register_transport_dbg(this, &Memory::transport_dbg);

    sc_core::SC_METHOD(readmem);
    sc_core::sensitive << clk.pos();
    sc_core::dont_initialize();

    sc_core::SC_METHOD(writemem);
    sc_core::sensitive << clk.pos();
    sc_core::dont_initialize();

    if (size > 0) {
      allocate(size);
      if (initptr) {
        std::copy(initptr, initptr + size, memdata.get());
      }
    }
  }

  virtual ~Memory() = default;

  // Object interface
  json hyperparams() const override {
    return {
      {"otype", "Memory"}, 
      {"size", memsize},
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
  void allocate(const std::size_t size) override {
    if (size == 0) {
      SC_REPORT_ERROR(name(), "Cannot allocate memory of size 0");
      return;
    }

    if (isinit && size == memsize)
      return;

    memdata = std::make_unique<T[]>(size);
    memsize = size;
    isinit = true;
  }

  void allocate(const std::shared_ptr<ParametricObject<T>>& target) override {
    allocate(static_cast<std::size_t>(target->n_params()));
  }

  T* data() const { return memdata.get(); }

  std::size_t size() const { return memsize; }

  void reset() {
    if (isinit) {
      std::fill(memdata.get(), memdata.get() + memsize, T{});
    }
  }

  bool is_valid_address(const std::size_t addr) const { 
    return addr < memsize; 
  }

  // Serialization
  json serialize() const override {
    return {
      {"size", memsize},
      {"data", std::vector<T>(memdata.get(), memdata.get() + memsize)}
    };
  }

  void deserialize(const json& data) override {
    if (data.contains("size") && data.contains("data")) {
      auto newsize = data["size"].get<std::size_t>();
      auto vec = data["data"].get<std::vector<T>>();

      if (newsize != vec.size()) {
        throw std::runtime_error("Size mismatch in deserialization");
      }

      allocate(newsize);
      std::copy(vec.begin(), vec.end(), memdata.get());
    }
  }

  void load_from_file(const std::filesystem::path& path, int offset = 0) override {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs)
      throw std::runtime_error("Cannot open file: " + path.string());

    ifs.seekg(offset);
    ifs.read(reinterpret_cast<char*>(memdata.get()), memsize * sizeof(T));
  }

  void save_to_file(const std::filesystem::path& path) override {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs)
      throw std::runtime_error("Cannot open file: " + path.string());

    ofs.write(reinterpret_cast<const char*>(memdata.get()), memsize * sizeof(T));
  }

 protected:
  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) override {
    auto cmd = trans.get_command();
    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    std::size_t len = static_cast<std::size_t>(trans.get_data_length());
    u8* ptr = trans.get_data_ptr();

    if (addr + len > memsize * sizeof(T) || addr % sizeof(T) != 0 || len % sizeof(T) != 0) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return;
    }

    T* base = reinterpret_cast<T*>(reinterpret_cast<u8*>(memdata.get()) + addr);

    if (cmd == tlm::TLM_READ_COMMAND) {
      std::memcpy(ptr, base, len);
    } else if (cmd == tlm::TLM_WRITE_COMMAND) {
      std::memcpy(base, ptr, len);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }

  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm_dmi& dmi_data) override {
    // Validate address alignment
    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    if (addr % sizeof(T) != 0) {
        SC_REPORT_ERROR(name(), "DMI request with unaligned address");
        return false;
    }

    // Check address range
    if (!is_valid_address(addr)) {
      std::ostringstream msg;
      msg << "DMI request out of range (addr=0x" << std::hex << addr 
          << ", max=0x" << (memsize * sizeof(T) - 1) << ")";
      SC_REPORT_WARNING(name(), msg.str().c_str());
      return false;
    }

    // Configure DMI region
    dmi_data.set_dmi_ptr(reinterpret_cast<u8*>(memdata.get()));
    dmi_data.set_start_address(0);
    dmi_data.set_end_address(memsize * sizeof(T) - 1);
    
    // A target wishing to deny read and write access to the DMI 
    // region should set the granted access type
    // to DMI_ACCESS_READ_WRITE, not to DMI_ACCESS_NONE
    dmi_data.allow_read_write();
    
    // Configure timing (example values - adjust as needed)
    dmi_data.set_read_latency(sc_core::sc_time(10, sc_core::SC_NS));
    dmi_data.set_write_latency(sc_core::sc_time(10, sc_core::SC_NS));
    
    // Optional: Set burst behavior
    dmi_data.set_burst_width(sizeof(T));  // Natural bus width
    dmi_data.set_burst_length(1);         // Single transfers by default

    return true;
  }

  std::size_t transport_dbg(tlm::tlm_generic_payload& trans) override {
    // Validate command type
    tlm::tlm_command cmd = trans.get_command();
    if (cmd != tlm::TLM_READ_COMMAND && cmd != tlm::TLM_WRITE_COMMAND) {
      trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
      return 0;
    }

    std::size_t addr = static_cast<std::size_t>(trans.get_address());
    u8* ptr = trans.get_data_ptr();
    std::size_t len = trans.get_data_length();
    
    if (addr >= memsize * sizeof(T)) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return 0;
    }

    // Calculate safe transfer length
    std::size_t remains = static_cast<std::size_t>(memsize * sizeof(T) - addr);
    std::size_t nbytes = (len < reamin) ? len : remains;

    if (nbytes == 0) {
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return 0;
    }

    if (cmd == tlm::TLM_READ_COMMAND) {
      std::memcpy(ptr, reinterpret_cast<u8*>(memdata.get()) + addr, nbytes);
    } 
    else { // TLM_WRITE_COMMAND
      std::memcpy(reinterpret_cast<u8*>(memdata.get()) + addr, ptr, nbytes);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
    return nbytes;
  }

  void readmem() {
    if (reset.read()) {
      outready.write(false);
      return;
    }

    if (ramen.read() && !writeen.read() && invalid.read()) {
      std::size_t rdaddr = addr.read();
      if (is_valid_address(rdaddr)) {
        outdata.write(memdata[rdaddr]);
        outready.write(true);
      } else {
        outready.write(false);
        sc_core::SC_REPORT_WARNING(name(), "Read addr out of bounds");
      }
    } else {
      outready.write(false);
    }
  }

  void writemem() {
    if (reset.read())
      return;

    if (ramen.read() && writeen.read() && invalid.read()) {
      std::size_t wraddr = addr.read();
      if (is_valid_address(wraddr)) {
        memdata[wraddr] = indata.read();
      } else {
        SC_REPORT_WARNING(name(), "Write addr out of bounds");
      }
    }
  }

 private:
  std::unique_ptr<T[]> memdata;
  std::size_t memsize;
  bool isinit;
};

}  // namespace adptsysc
