auto RDRAM::Debugger::load(Node::Object parent) -> void {
  memory.ram = parent->append<Node::Debugger::Memory>("RDRAM");
  if(!system.expansionPak) {
    memory.ram->setSize(4_MiB); 
  } else {
    memory.ram->setSize(4_MiB + 4_MiB);
  }
  
  memory.ram->setRead([&](u32 address) -> u8 {
    return rdram.ram.read<Byte>(address, RBusDevice::ARES_DEBUGGER);
  });
  memory.ram->setWrite([&](u32 address, u8 data) -> void {
    return rdram.ram.write<Byte>(address, data, RBusDevice::ARES_DEBUGGER);
  });

  memory.dcache = parent->append<Node::Debugger::Memory>("DCache");
  memory.dcache->setSize(4_MiB + 4_MiB);
  memory.dcache->setRead([&](u32 address) -> u8 {
    u32 vaddr = address | 0x8000'0000;
    return cpu.dcache.readDebug<Byte>(vaddr, address);
  });
  memory.dcache->setWrite([&](u32 address, u8 data) -> void {
    u32 vaddr = address | 0x8000'0000;
    auto& line = cpu.dcache.line(vaddr);
    if(line.hit(address)) {
      line.write<Byte>(address, data);
    } else {
      rdram.ram.write<Byte>(address, data, RBusDevice::ARES_DEBUGGER);
    }
  });

  tracer.io = parent->append<Node::Debugger::Tracer::Notification>("I/O", "RDRAM");
}

auto RDRAM::Debugger::io(bool mode, u32 chipID, u32 address, u32 data) -> void {
  static const std::vector<string> registerNames = {
    "RDRAM_DEVICE_TYPE",
    "RDRAM_DEVICE_ID",
    "RDRAM_DELAY",
    "RDRAM_MODE",
    "RDRAM_REF_INTERVAL",
    "RDRAM_REF_ROW",
    "RDRAM_RAS_INTERVAL",
    "RDRAM_MIN_INTERVAL",
    "RDRAM_ADDRESS_SELECT",
    "RDRAM_DEVICE_MANUFACTURER",
  };

  if(unlikely(tracer.io->enabled())) {
    string message;
    string name = (address < registerNames.size() ? registerNames[address] : string("RDRAM_UNKNOWN"));
    name.append("[", chipID, "]");
    if(mode == Read) {
      message = {nall::split(name, "|").front(), " => ", hex(data, 8L)};
    }
    if(mode == Write) {
      message = {nall::split(name, "|").back(), " <= ", hex(data, 8L)};
    }
    tracer.io->notify(message);
  }
}

auto RDRAM::Debugger::cacheErrorContext(string peripheral) -> string {
  if(peripheral == "CPU") {
    return { "\tCurrent CPU PC: 0x", hex(cpu.ipu.pc, 16L), "\n" };
  }
  if(peripheral == "SP DMA") {
    if(rsp.dma.current.originCpu) {
      return { "\tSP DMA started at CPU PC: 0x", hex(rsp.dma.current.originPc, 16L), "\n" };
    } else {
      return { "\tSP DMA started at RSP PC: 0x", hex(rsp.dma.current.originPc,  3L), "\n" };
    }
  }
  if(peripheral == "PI DMA") {
    return { "\tPI DMA started at CPU PC: 0x", hex(pi.io.originPc, 16L), "\n" };
  }
  if(peripheral == "AI DMA") {
    return { "\tAI DMA started at CPU PC: 0x", hex(ai.io.dmaOriginPc[0], 16L), "\n" };
  }
  return "";
}

auto RDRAM::Debugger::readWord(u32 address, int size, RBusDevice device) -> void {
  if (system.homebrewMode && (address & ~15) != lastReadCacheline) {
    lastReadCacheline = address & ~15;
    auto& line = cpu.dcache.line(address);
    u16 dirtyMask = ((1 << size) - 1) << (address & 0xF);
    if (line.hit(address) && (line.dirty & dirtyMask)) {
      if(device == RBusDevice::SP_DMA) return; // ignore: prefer deferring to actual DMEM access, see RSP::Debugger::dmemReadWord
      string msg = { rdram.requestorName(device), " reading from RDRAM address 0x", hex(address), " which is modified in the cache (missing cache writeback?)\n"};
      msg.append(string{ "\tCacheline was loaded at CPU PC: 0x", hex(line.fillPc, 16L), "\n" });
      msg.append(string{ "\tCacheline was last written at CPU PC: 0x", hex(line.dirtyPc, 16L), "\n" });
      msg.append(cacheErrorContext(rdram.requestorName(device)));
      debug(unusual, msg);
    }
  }
}

auto RDRAM::Debugger::stored(u32 address, u16 bytes, u64 pc) -> void {
  if(lateStores.empty()) lateStores.resize(rdram.ram.size >> 4);
  u32 line = address >> 4;
  if(line >= lateStores.size()) return;
  auto& store = lateStores[line];
  if(store.clock != clock) store.bytes = 0;
  store.clock = clock;
  store.bytes |= bytes;
  store.pc = pc;
}

auto RDRAM::Debugger::changedBytes(u32 address, const u32* words, u16 bytes) -> u16 {
  u16 changed = 0;
  for(u32 byte : range(16)) {
    if(!(bytes >> byte & 1)) continue;
    u8 cached = words[byte >> 2] >> (24 - (byte & 3) * 8);
    if(cached != rdram.ram.Nintendo64::Memory::Writable::read<Byte>(address + byte)) changed |= 1 << byte;
  }
  return changed;
}

auto RDRAM::Debugger::writtenBack(u32 address, const u32* words, u16 dirty, u64 pc) -> void {
  if(u16 changed = changedBytes(address, words, dirty)) stored(address, changed, pc);
}

auto RDRAM::Debugger::queued(u32 list) -> void {
  queuedLists.push_back({list & 0x1fff'ffff, ++clock});
}

//OSTask, which osSpTaskLoad copies to DMEM before the RSP starts, gives the
//task's type, whether it resumes a yielded one and its display list
auto RDRAM::Debugger::rspStarted() -> void {
  static constexpr u32 TaskAddress = 0xfc0;
  static constexpr u32 DataPointer = 0x30;
  static constexpr u32 GraphicsTask = 1;
  static constexpr u32 TaskYielded = 1;
  u32 type = rsp.dmem.read<Word>(TaskAddress);
  u32 flags = rsp.dmem.read<Word>(TaskAddress + 4);
  if(type == GraphicsTask && (flags & TaskYielded)) {
    rspSince = graphicsSince;
    return;
  }
  rspSince = ++clock;
  if(type != GraphicsTask) return;
  u32 address = rsp.dmem.read<Word>(TaskAddress + DataPointer) & 0x1fff'ffff;
  for(auto list = queuedLists.begin(); list != queuedLists.end(); list++) {
    if(list->address != address) continue;
    rspSince = list->clock;
    queuedLists.erase(list);
    break;
  }
  graphicsSince = rspSince;
}

auto RDRAM::Debugger::lateStore(u32 address, u16 bytes, u32 since) -> const LateStore* {
  u32 line = address >> 4;
  if(line >= lateStores.size()) return nullptr;
  auto& store = lateStores[line];
  if(store.clock == 0 || store.clock < since || !(store.bytes & bytes)) return nullptr;
  return &store;
}

auto RDRAM::Debugger::writeWord(u32 address, int size, u64 value, RBusDevice device) -> void {
  if (system.homebrewMode && device == RBusDevice::VR4300_UNCACHED) {
    u16 changed = 0;
    for(u32 byte : range(size)) {
      u8 newByte = value >> ((size - 1 - byte) * 8);
      if(newByte != rdram.ram.Nintendo64::Memory::Writable::read<Byte>(address + byte)) changed |= 1 << ((address + byte) & 0xF);
    }
    if(changed) stored(address & ~15, changed, cpu.ipu.pc);
  }
  if (system.homebrewMode && (address & ~15) != lastWrittenCacheline) {
    lastWrittenCacheline = address & ~15;
    auto& line = cpu.dcache.line(address);
    if (line.hit(address)) {
      string msg = { rdram.requestorName(device), " writing to RDRAM address 0x", hex(address), " which is cached (missing cache invalidation?)\n"};
      msg.append(string{ "\tCacheline was loaded at CPU PC: 0x", hex(line.fillPc, 16L), "\n" });
      msg.append(cacheErrorContext(rdram.requestorName(device)));
      debug(unusual, msg);
    }
  }
}
