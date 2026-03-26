#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <deque>
#include <memory>
#include <adptsysc/object.hh>

namespace adptsysc {

template <typename E> struct Context;

template <typename E>
class SyscMemory : public ObjectWithMutableHyperparams, 
                   public sc_core::sc_module {
public:
  using T = typename E::Eval_T;

  // Static creation function following OutputFile pattern
  static std::unique_ptr<SyscMemory<E>> 
  create(Context<E> &ctx, sc_core::sc_module_name name,
         std::size_t size = 0, T* initptr = nullptr);

  // Static function for testbed simulation
  static bool run_testbench(Context<E> &ctx);

  tlm_utils::simple_target_socket<SyscMemory> targ_socket;
  
  
  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;
  
  void allocate(std::size_t size);
  void allocate(const std::shared_ptr<ParametricObject<T>>& target);
  
  T* data() const { return mem_data.get(); }
  std::size_t size() const { return mem_size; }
  
  void mem_reset(Context<E> &ctx);

  bool is_valid_addr(const std::size_t addr) const { 
    return addr < mem_size; 
  }

  // Serialization
  json serialize(Context<E> &ctx) const;
  void deserialize(Context<E> &ctx, const json& data);

  // File I/O operations using Context<E>
  void load_from_file(Context<E> &ctx);
  void save_to_file(Context<E> &ctx);
  void save_to_text_file(Context<E> &ctx);
  void load_from_text_file(Context<E> &ctx);

  // Memory dump for debugging
  void dump_memory(Context<E> &ctx, const std::string& desc = "") const;
  virtual ~SyscMemory() = default;

protected:
  // Protected constructor - use create() static method
  SyscMemory(sc_core::sc_module_name name, 
             Context<E> &ctx,
             const std::size_t size = 0, 
             T* initptr = nullptr);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi_data);
  unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

private:
  sc_core::sc_time read_delay;
  sc_core::sc_time write_delay;
  std::unique_ptr<T[]> mem_data;
  std::size_t mem_size;
  bool is_init;
};

}