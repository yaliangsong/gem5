#include "base/mem_data.hh"

#define IS_ALIGNED_ADDR(x, a)	(((x) & ((Addr)(a) - 1)) == 0)
#define MAX_MEM_DATA_SIZE	64

bool
MemData::merge(MemData &data)
{
    if (size < MAX_MEM_DATA_SIZE &&
        data.addr == addr + size &&
        data.size == size &&
        IS_ALIGNED_ADDR(addr, size << 1)) {
        value |= (data.value << (8 * size));
        size <<= 1;
        return true;
    }
    return false;
}
