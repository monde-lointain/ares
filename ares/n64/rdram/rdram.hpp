//RAMBUS RAM

struct RDRAM : Memory::RCP<RDRAM> {
  Node::Object node;

  struct Writable : public Memory::Writable {
    RDRAM& self;

    Writable(RDRAM& self) : self(self) {}

    //Lets queued RDP commands that may use the memory finish first
    auto waitForRdp(u32 address, u32 length) -> void {
      #if defined(VULKAN)
      vulkan.access(address, length);
      #endif
    }

    template<u32 Size>
    auto read(u32 address, RBusDevice device) -> u64 {
      if (address >= size) return 0;
      waitForRdp(address, Size);
      if (unlikely(system.homebrewMode)) {
        self.debugger.readWord(address, Size, device);
        self.profile.metrics[(u32)device].reads += Size;
      }
      return Memory::Writable::read<Size>(address);
    }

    template<u32 Size>
    auto writeRepeat(u32 address, u64 value, u8 length) -> void {
      if constexpr(Size == Byte) {
        value = value & (0xFFFFFFFF >> (24 - (address & 3) * 8));
        value = (u32)((value << 24) | (value >> 8));
      } else if constexpr(Size == Half) {
        value = value & (0xFFFFFFFF >> (16 - (address & 2) * 8));
        value = (u32)((value << 16) | (value >> 16));
      }
      if constexpr(Size != Dual)
        value = (value << 32) | (u32) value;

      const u32 end = min((address & ~7) + length, size);
      if (end <= address)
        return;

      length = end - address;

      if (address & 1) {
        Memory::Writable::write<Byte>(address, value >> 56);
        value = (value << 8) | (value >> 56);
        address = (address & ~0x7FF) | ((address + 1) & 0x7FF);
        length -= 1;
      }
      if ((address & 2) && length >= 2) {
        Memory::Writable::write<Half>(address, value >> 48);
        value = (value << 16) | (value >> 48);
        address = (address & ~0x7FF) | ((address + 2) & 0x7FF);
        length -= 2;
      }
      if ((address & 4) && length >= 4) {
        Memory::Writable::write<Word>(address, value >> 32);
        value = (value << 32) | (value >> 32);
        address = (address & ~0x7FF) | ((address + 4) & 0x7FF);
        length -= 4;
      }

      while (length >= 8) {
        Memory::Writable::write<Dual>(address, value);
        address = (address & ~0x7FF) | ((address + 8) & 0x7FF);
        length -= 8;
      }
      if (length >= 4) {
        Memory::Writable::write<Word>(address, value >> 32);
        value <<= 32;
        address += 4;
        length -= 4;
      }
      if (length >= 2) {
        Memory::Writable::write<Half>(address, value >> 48);
        value <<= 16;
        address += 2;
        length -= 2;
      }
      if (length == 1)
        Memory::Writable::write<Byte>(address, value >> 56);
    }

    template<u32 Size>
    auto write(u32 address, u64 value, RBusDevice device) -> void {
      if (address >= size) return;
      waitForRdp(address, unlikely(mi.initializeMode()) ? mi.initializeLength() + 1 : Size);
      if (unlikely(system.homebrewMode)) {
        self.debugger.writeWord(address, Size, value, device);
      }
      if (unlikely(mi.initializeMode())) {
        u32 len = mi.initializeLength() + 1;
        writeRepeat<Size>(address, value, len);
        if(unlikely(system.homebrewMode)) {
          self.profile.metrics[(u32)device].writes += len;
        }
      } else {
        Memory::Writable::write<Size>(address, value);
        if(unlikely(system.homebrewMode)) {
          self.profile.metrics[(u32)device].writes += Size;
        }
      }
    }

    template<u32 Size>
    auto writeBurst(u32 address, u32 *value, RBusDevice device) -> void {
      if (address >= size) return;
      waitForRdp(address, Size);
      if (unlikely(system.homebrewMode)) {
        self.profile.metrics[(u32)device].writes += Size;
      }
      Memory::Writable::write<Word>(address | 0x00, value[0]);
      Memory::Writable::write<Word>(address | 0x04, value[1]);
      Memory::Writable::write<Word>(address | 0x08, value[2]);
      Memory::Writable::write<Word>(address | 0x0c, value[3]);
      if (Size == ICache) {
        Memory::Writable::write<Word>(address | 0x10, value[4]);
        Memory::Writable::write<Word>(address | 0x14, value[5]);
        Memory::Writable::write<Word>(address | 0x18, value[6]);
        Memory::Writable::write<Word>(address | 0x1c, value[7]);
      }
    }

    template<u32 Size>
    auto readBurst(u32 address, u32 *value, RBusDevice device) -> void {
      if (address >= size) {
        value[0] = value[1] = value[2] = value[3] = 0;
        if (Size == ICache)
          value[4] = value[5] = value[6] = value[7] = 0;
        return;
      }
      waitForRdp(address, Size);
      if (unlikely(system.homebrewMode)) {
        self.profile.metrics[(u32)device].reads += Size;
      }
      value[0] = Memory::Writable::read<Word>(address | 0x00);
      value[1] = Memory::Writable::read<Word>(address | 0x04);
      value[2] = Memory::Writable::read<Word>(address | 0x08);
      value[3] = Memory::Writable::read<Word>(address | 0x0c);
      if (Size == ICache) {
        value[4] = Memory::Writable::read<Word>(address | 0x10);
        value[5] = Memory::Writable::read<Word>(address | 0x14);
        value[6] = Memory::Writable::read<Word>(address | 0x18);
        value[7] = Memory::Writable::read<Word>(address | 0x1c);
      }
    }

  } ram{*this};

  struct Debugger {
    u32 lastReadCacheline    = 0xffff'ffff;
    u32 lastWrittenCacheline = 0xffff'ffff;

    //debugger.cpp
    auto load(Node::Object) -> void;
    auto io(bool mode, u32 chipID, u32 address, u32 data) -> void;
    auto readWord(u32 address, int size, RBusDevice device) -> void;
    auto writeWord(u32 address, int size, u64 value, RBusDevice device) -> void;
    auto cacheErrorContext(string device) -> string;

    //Late stores: CPU stores that reached RDRAM while the RSP or RDP may
    //read them, so that what they read depends on timing. A clock counts
    //events: RSP tasks started (not resumed) and, when a driver marks them,
    //display lists queued. Each 16-byte line keeps the clock at its last
    //writeback, eviction or uncached store, with the bytes it changed.
    //A task's reads are late from the event that started its exposure:
    //the queueing of its list if marked, else its start.
    struct LateStore {
      u64 pc = 0;
      u32 clock = 0;  //0 means no store
      u16 bytes = 0;  //a bit per byte of the line
    };
    std::vector<LateStore> lateStores;
    u32 clock = 0;
    u32 rspSince = 0;  //the exposure of the task the RSP runs
    u32 graphicsSince = 0;  //that of the last graphics task, which the RDP draws
    struct QueuedList {
      u32 address;
      u32 clock;  //at its queueing
    };
    std::vector<QueuedList> queuedLists;  //awaiting their task's start
    //Coprocessor reads of late stores or of lines dirty in the CPU's cache,
    //either of which a timing change can alter; unlike notices, every one
    u64 racedReads = 0;
    auto stored(u32 address, u16 bytes, u64 pc) -> void;
    //Which of the BYTES of a cache line's WORDS differ from RDRAM's at
    //ADDRESS; rewriting a byte with its value races nothing
    auto changedBytes(u32 address, const u32* words, u16 bytes) -> u16;
    auto writtenBack(u32 address, const u32* words, u16 dirty, u64 pc) -> void;
    //Marks a display list queued; call it once the CPU has written back the
    //data the list uses, as stores before then are not late
    auto queued(u32 list) -> void;
    auto rspStarted() -> void;
    //Notes the RSP signaling that its task yielded
    auto rspYielded() -> void;
    bool graphicsYielded = false;  //the last graphics task yielded
    //The late store a read of a line's BYTES sees for a task exposed SINCE
    auto lateStore(u32 address, u16 bytes, u32 since) -> const LateStore*;

    struct Memory {
      Node::Debugger::Memory ram;
      Node::Debugger::Memory dcache;
    } memory;

    struct Tracer {
      Node::Debugger::Tracer::Notification io;
    } tracer;
  } debugger;

  //rdram.cpp
  auto requestorName(RBusDevice requestor) -> const char* {
    switch(requestor) {
      case RBusDevice::VR4300_ICACHE:   return "VR4300 ICache";
      case RBusDevice::VR4300_DCACHE:   return "VR4300 DCache";
      case RBusDevice::VR4300_UNCACHED: return "VR4300 Uncached";
      case RBusDevice::SP_DMA:          return "SP DMA";
      case RBusDevice::PI_DMA:          return "PI DMA";
      case RBusDevice::SI_DMA:          return "SI DMA";
      case RBusDevice::VI_DMA:          return "VI DMA";
      case RBusDevice::AI_DMA:          return "AI DMA";
      case RBusDevice::DP_DMA:          return "DP DMA";
      case RBusDevice::DP_DRAW:         return "DP Draw";
      case RBusDevice::ARES_DEBUGGER:   return "Ares Debugger";
      case RBusDevice::ARES_JIT:        return "Ares JIT";
      case RBusDevice::ARES_IPL3:       return "Ares IPL3";
      default:                          return "Unknown";
    }
  }

  struct Metric {
    u64 reads, writes;
    auto total() const -> u64 { return reads + writes; }
  };
  
  struct Profile {
    Metric metrics[(u32)RBusDevice::NUM_RBUS_DEVICES];

    auto total() -> Metric {
      Metric total;
      for(u32 n = 0; n < (u32)RBusDevice::NUM_RBUS_HW_DEVICES; n++) {
        total.reads  += metrics[n].reads;
        total.writes += metrics[n].writes;
      }
      return total;
    }
  } profile;

  auto load(Node::Object) -> void;
  auto unload() -> void;
  auto power(bool reset) -> void;

  //io.cpp
  auto readWord(u32 address, Thread& thread) -> u32;
  auto writeWord(u32 address, u32 data, Thread& thread) -> void;

  //serialization.cpp
  auto serialize(serializer&) -> void;

  struct Chip {
    n32 deviceType;
    n32 deviceID;
    n32 delay;
    n32 mode;
    n32 refreshInterval;
    n32 refreshRow;
    n32 rasInterval;
    n32 minInterval;
    n32 addressSelect;
    n32 deviceManufacturer;
    n32 currentControl;
  } chips[4];
};

extern RDRAM rdram;
