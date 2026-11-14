#include "rdp_device.hpp"

namespace ares::Nintendo64 {

struct Vulkan {
  auto load(Node::Object) -> bool;
  auto unload() -> void;

  auto render() -> bool;
  auto frame() -> void;
  auto writeWord(u32 address, u32 data) -> void;
  auto scanoutAsync(bool field) -> bool;
  auto mapScanoutRead(const u8*& rgba, u32& width, u32& height) -> void;
  auto unmapScanoutRead() -> void;
  auto endScanout() -> void;
  auto crashed() -> const char*;
  auto traceCommand(const u32* words, u32 length) -> void;

  struct Implementation;
  Implementation* implementation = nullptr;

  bool enable = true;
  bool disableVideoInterfaceProcessing = false;
  bool weaveDeinterlacing = false;
  // Makes the RDP appear to read and write RDRAM at the emulated time its
  // commands were issued rather than whenever the host GPU gets to them, so
  // runs are reproducible. An emulated access to memory that queued
  // commands may still use waits for the GPU first; see Footprint.
  bool synchronous = false;
  u32  internalUpscale = 1;  //1, 2, 4, 8
  bool supersampleScanout = false;
  u32  outputUpscale = supersampleScanout ? 1 : internalUpscale;

  // An image the RDP reads or writes, as set by the last Set*Image command
  struct Image {
    u32 address = 0;
    u32 size = 0;   // log2 of the texel width in nibbles: 0 = 4 bits
    u32 width = 0;  // in texels
  };
  Image textureImage;  // read by texture loads
  Image colorImage;    // drawn to
  Image depthImage;    // its width and texel size are the color image's
  u32 scissorRows = 0; // rows from 0 that the scissor box lets draws reach

  // RDRAM that commands sent to the GPU may still read or write, in blocks.
  // It is a superset: textures loaded, plus the color and depth images up
  // to the scissor box. Every emulated RDRAM access checks it (see RDRAM),
  // so it must be cheap to test while empty.
  struct Footprint {
    static constexpr u32 BlockShift = 8;
    static constexpr u32 Blocks = 0x80'0000 >> BlockShift;  //8 MB of RDRAM

    auto touches(u32 address, u32 length) const -> bool;
    auto mark(u32 address, u32 length) -> void;
    auto clear() -> void;

    bool pending = false;       // some block is marked
    bool imagesMarked = false;  // since the last change to the images
    u32 low = Blocks;           // marked blocks lie in [low, high]
    u32 high = 0;
    u8 blocks[Blocks] = {};
  } footprint;

  auto trackImages(const u32* words, u32 code) -> void;
  auto loadBytes(const u32* words, u32 code) const -> maybe<u32>;
  auto markFootprint(const u32* words, u32 code) -> void;
  auto waitForQueuedCommands() -> void;

  // Called before every emulated RDRAM access
  auto access(u32 address, u32 length) -> void {
    if(unlikely(footprint.pending) && footprint.touches(address, length)) {
      waitForQueuedCommands();
    }
  }
};

extern Vulkan vulkan;

}
