
#ifndef __CPU_MEM_DATA_HH__
#define __CPU_MEM_DATA_HH__

#include <memory>
#include <vector>

#include "arch/types.hh"
#include "base/types.hh"

class MemData {
  public:
    Addr addr;
    Addr size;
    uint64_t value;

  public:
    MemData(Addr _addr, uint8_t _value)
      : addr(_addr), size(1), value((uint64_t)_value) {}
    bool merge(MemData &data);
};

#endif /* __CPU_MEM_DATA_HH__ */
