// n64-harness: a headless, scriptable Nintendo 64 runner for on-target tests.
//
// It loads a cartridge without any GUI, audio or input driver and reads
// newline-terminated commands from stdin. Each command writes exactly one
// line to stdout, so a test driver can talk to it through a pipe:
//
//   break ADDR          stop when the CPU reaches ADDR          -> ok
//   unbreak ADDR        remove a breakpoint                     -> ok
//   watch ADDR LEN      stop after the CPU stores to any of the
//                       LEN bytes from ADDR; DMA does not count -> ok
//   unwatch ADDR LEN    remove a watchpoint                     -> ok
//   run MAXFRAMES       run until a breakpoint or MAXFRAMES video
//                       frames pass  -> "halt PC FRAMES" | "timeout FRAMES"
//                       An interrupt taken at a breakpoint's instruction
//                       returns to it, so the same breakpoint can halt
//                       twice for one execution; a driver that counts
//                       calls must tell them apart by the program's state.
//   read ADDR LEN       read memory through the CPU's view      -> hex bytes
//   write ADDR HEX      write memory through the CPU's view     -> ok
//   dump PATH ADDR LEN [ADDR LEN]...
//                       write the ranges' raw bytes to PATH, one
//                       after another, for snapshots too big for
//                       hex                                     -> ok BYTES
//   reg INDEX           read a GDB-numbered register            -> hex
//   regs                read all 71 GDB registers               -> hex
//   setreg INDEX HEX    write a GDB-numbered register           -> ok
//   input PORT NAME V   set a gamepad input (button 0/1, axis
//                       -32768..32767); PORT counts from 1      -> ok
//   screenshot PATH     write the last video frame as a PPM     -> ok
//   video on|off        scan out and convert each video frame, as
//                       at start, or skip it; neither changes the
//                       machine, and without video, runs take
//                       about a tenth less CPU time, but
//                       screenshots go stale                    -> ok
//   trace COMPONENT NAME on|off
//                       collect a tracer's messages, e.g. "RDP Command"
//                                                               -> ok
//   tracetake PATH      write the collected messages, one a line, and
//                       start collecting afresh                 -> ok COUNT
//   races               with --homebrew, how many times the RSP or
//                       RDP has read bytes the CPU stored while the
//                       reading task ran or still held in its data
//                       cache, so far                 -> decimal count
//   queued ADDR         with --homebrew, mark the display list at ADDR
//                       queued, after the CPU wrote back its data, so
//                       stores from then on count as racing it  -> ok
//   graphicsdelay US    with --homebrew, hold each graphics task US
//                       microseconds before it runs, as an RDP that
//                       falls behind would, so that stores racing it
//                       come before its reads and count   -> ok
//   quit                exit
//
// Addresses are hexadecimal virtual addresses (e.g. 80025c00). Errors are
// reported as "error MESSAGE" so the driver can fail the test loudly.
//
// Usage: n64-harness [--sram FILE] [--homebrew] [--no-spin-skip] ROM
//
// --homebrew turns on ares's Homebrew Mode, whose checks report, among
// others, a DMA reading memory still dirty in the CPU's data cache and one
// writing memory the cache holds. ares prints such notices, like all its
// messages, to stdout; the harness sends them to stderr and keeps stdout
// for the replies alone. It also counts the reads that raced a CPU store
// (see "races"), each of which can make the frame depend on timing.
//
// The harness skips the repeats of loops that only read memory and
// compute, such as a game polling for a frame's end, which the machine
// would only repeat; --no-spin-skip runs each of them, for comparison.

#include <nall/nall.hpp>
#include <nall/main.hpp>
#include <nall/gdb/server.hpp>
using namespace nall;

#include <ares/ares.hpp>
#include <mia/mia.hpp>

#include <iostream>
#include <unistd.h>
#include <mutex>

// The core's entry points are not exported through a header.
namespace ares::Nintendo64 {
  auto load(Node::System& node, string name) -> bool;
  auto option(string name, string value) -> bool;
  auto racedReads() -> u64;
  auto queued(u32 list) -> void;
  auto delayGraphicsTasks(u32 microseconds) -> void;
  auto outputVideo(bool enabled) -> void;
}

namespace {

constexpr u32 controllerCount = 4;

struct Harness : ares::Platform {
  ares::Node::System root;
  std::shared_ptr<mia::Pak> system;
  std::shared_ptr<mia::Pak> game;
  map<string, s32> inputs;  // "PORT/NAME" -> value

  std::mutex videoMutex;
  std::vector<u32> lastFrame;
  u32 lastFrameWidth = 0;
  u32 lastFrameHeight = 0;
  std::vector<string> traceMessages;

  FILE* replies = stdout;

  auto load(const string& romPath, const string& sramPath, bool homebrew, bool skipsSpins) -> bool;
  auto serve() -> void;

  auto attach(ares::Node::Object) -> void override {}
  auto pak(ares::Node::Object node) -> std::shared_ptr<vfs::directory> override;
  auto video(ares::Node::Video::Screen, const u32* data, u32 pitch, u32 width, u32 height) -> void override;
  auto audio(ares::Node::Audio::Stream stream) -> void override;
  auto input(ares::Node::Input::Input node) -> void override;
  auto log(ares::Node::Debugger::Tracer::Tracer node, string_view message) -> void override;

private:
  auto runUntilHalt(u64 maxFrames) -> string;
  auto writeScreenshot(const string& path) -> bool;
  auto setTrace(const string& component, const string& name, bool enabled) -> bool;
  auto takeTrace(const string& path) -> string;
  auto dumpMemory(const std::vector<string>& words) -> string;
  auto execute(const std::vector<string>& words) -> string;
};

auto Harness::pak(ares::Node::Object node) -> std::shared_ptr<vfs::directory> {
  if(node->name() == "Nintendo 64") return system->pak;
  if(node->name() == "Nintendo 64 Cartridge") return game->pak;
  return {};
}

auto Harness::video(ares::Node::Video::Screen, const u32* data, u32 pitch, u32 width, u32 height) -> void {
  std::lock_guard guard{videoMutex};
  lastFrame.resize(width * height);
  for(u32 y : range(height)) {
    memory::copy<u32>(lastFrame.data() + y * width, data + y * (pitch / sizeof(u32)), width);
  }
  lastFrameWidth = width;
  lastFrameHeight = height;
}

// Audio is discarded, but the streams must be drained or they grow forever.
auto Harness::audio(ares::Node::Audio::Stream stream) -> void {
  f64 samples[8];
  while(stream->pending()) stream->read(samples);
}

auto Harness::input(ares::Node::Input::Input node) -> void {
  auto device = ares::Node::parent(node);
  if(!device) return;
  auto port = ares::Node::parent(device);
  if(!port) return;

  s32 value = 0;
  if(auto found = inputs.find({port->name(), "/", node->name()})) value = found();
  if(auto button = node->cast<ares::Node::Input::Button>()) button->setValue(value != 0);
  if(auto axis = node->cast<ares::Node::Input::Axis>()) axis->setValue(value);
}

auto Harness::log(ares::Node::Debugger::Tracer::Tracer, string_view message) -> void {
  traceMessages.push_back(message);
}

auto Harness::load(const string& romPath, const string& sramPath, bool homebrew, bool skipsSpins) -> bool {
  system = mia::System::create("Nintendo 64");
  if(system->load() != successful) return print(stderr, "error: system load failed\n"), false;
  game = mia::Medium::create("Nintendo 64");
  if(game->load(romPath) != successful) return print(stderr, "error: cannot load ", romPath, "\n"), false;

  // Saves never persist; a run starts from a blank or a supplied SRAM image.
  if(sramPath) {
    auto image = file::read(sramPath);
    auto sram = game->pak->write("save.ram");
    if(image.empty() || !sram) return print(stderr, "error: cannot load SRAM ", sramPath, "\n"), false;
    sram->write({image.data(), min((u64)image.size(), sram->size())});
  }

  ares::Nintendo64::option("Enable GPU acceleration", true);
  ares::Nintendo64::option("Expansion Pak", true);
  ares::Nintendo64::option("Recompiler", true);
  ares::Nintendo64::option("Synchronous RDP", true);
  ares::Nintendo64::option("Homebrew Mode", homebrew);
  ares::Nintendo64::option("Skip Spin Loops", skipsSpins);
  nall::GDB::server.reset();
  if(!ares::Nintendo64::load(root, "[Nintendo] Nintendo 64 (NTSC)")) return false;

  if(auto port = root->find<ares::Node::Port>("Cartridge Slot")) {
    port->allocate();
    port->connect();
  }
  for(u32 id : range(controllerCount)) {
    if(auto port = root->find<ares::Node::Port>({"Controller Port ", 1 + id})) {
      port->allocate("Gamepad");
      port->connect();
    }
  }

  root->power();
  nall::GDB::server.attachLocalClient();
  return true;
}

auto Harness::runUntilHalt(u64 maxFrames) -> string {
  auto& server = nall::GDB::server;
  u64 frames = 0;
  if(server.isHalted()) server.resumeLocalClient();
  while(frames < maxFrames) {
    root->run();
    if(server.isHalted()) {
      return {"halt ", hex(server.currentProgramCounter(), 8L), " ", frames};
    }
    frames++;  // run() returns once per video refresh when not halted
  }
  return {"timeout ", frames};
}

auto Harness::writeScreenshot(const string& path) -> bool {
  std::lock_guard guard{videoMutex};
  file_buffer fp{path, file::mode::write};
  if(!fp) return false;
  fp.writes(string{"P6\n", lastFrameWidth, " ", lastFrameHeight, "\n255\n"});
  for(u32 pixel : lastFrame) {
    fp.write(pixel >> 16);
    fp.write(pixel >> 8);
    fp.write(pixel >> 0);
  }
  return true;
}

auto Harness::setTrace(const string& component, const string& name, bool enabled) -> bool {
  for(auto& tracer : root->find<ares::Node::Debugger::Tracer::Tracer>()) {
    if(tracer->component() != component || tracer->name() != name) continue;
    tracer->setPrefix(false);
    tracer->setAutoLineBreak(false);
    tracer->setTerminal(enabled);  // enables it; log() captures the output
    return true;
  }
  return false;
}

auto Harness::takeTrace(const string& path) -> string {
  file_buffer fp{path, file::mode::write};
  if(!fp) return "error cannot write trace";
  for(auto& message : traceMessages) {
    fp.writes(message);
    fp.write('\n');
  }
  auto count = traceMessages.size();
  traceMessages.clear();
  return {"ok ", count};
}

auto Harness::dumpMemory(const std::vector<string>& words) -> string {
  auto& server = nall::GDB::server;
  file_buffer fp{words[1], file::mode::write};
  if(!fp) return "error cannot write dump";
  u64 total = 0;
  for(u32 index = 2; index < words.size(); index += 2) {
    auto text = server.hooks.read(words[index].hex(), words[index + 1].hex());
    if(!text) return "error unreadable address";
    for(u32 i = 0; i + 1 < text.size(); i += 2) fp.write(text.slice(i, 2).hex());
    total += text.size() / 2;
  }
  return {"ok ", total};
}

auto Harness::execute(const std::vector<string>& words) -> string {
  auto& server = nall::GDB::server;
  auto count = words.size();
  auto& name = words[0];

  if(name == "break" && count == 2) return server.setBreakpoint(words[1].hex(), true), "ok";
  if(name == "unbreak" && count == 2) return server.setBreakpoint(words[1].hex(), false), "ok";
  if(name == "watch" && count == 3) return server.setWriteWatchpoint(words[1].hex(), words[2].hex(), true), "ok";
  if(name == "unwatch" && count == 3) return server.setWriteWatchpoint(words[1].hex(), words[2].hex(), false), "ok";
  if(name == "queued" && count == 2) return ares::Nintendo64::queued(words[1].hex()), "ok";
  if(name == "graphicsdelay" && count == 2) return ares::Nintendo64::delayGraphicsTasks(words[1].natural()), "ok";
  if(name == "races" && count == 1) return string{ares::Nintendo64::racedReads()};
  if(name == "run" && count == 2) return runUntilHalt(words[1].natural());
  if(name == "read" && count == 3) {
    auto bytes = server.hooks.read(words[1].hex(), words[2].hex());
    return bytes ? bytes : string{"error unreadable address"};
  }
  if(name == "dump" && count >= 4 && count % 2 == 0) return dumpMemory(words);
  if(name == "write" && count == 3) {
    auto& text = words[2];
    if(text.size() % 2) return "error odd hex length";
    std::vector<u8> data;
    for(u32 i = 0; i < text.size(); i += 2) data.push_back(text.slice(i, 2).hex());
    server.hooks.write(words[1].hex(), data);
    return "ok";
  }
  if(name == "reg" && count == 2) return server.hooks.regRead(words[1].natural());
  if(name == "regs" && count == 1) return server.hooks.regReadGeneral();
  if(name == "setreg" && count == 3) {
    return server.hooks.regWrite(words[1].natural(), words[2].hex()) ? "ok" : "error bad register";
  }
  if(name == "input" && count == 4) {
    inputs.insert({"Controller Port ", words[1], "/", words[2]}, words[3].integer());
    return "ok";
  }
  if(name == "trace" && count == 4 && (words[3] == "on" || words[3] == "off")) {
    return setTrace(words[1], words[2], words[3] == "on") ? "ok" : "error no such tracer";
  }
  if(name == "tracetake" && count == 2) return takeTrace(words[1]);
  if(name == "video" && count == 2 && (words[1] == "on" || words[1] == "off")) {
    return ares::Nintendo64::outputVideo(words[1] == "on"), "ok";
  }
  if(name == "screenshot" && count == 2) {
    return writeScreenshot(words[1]) ? "ok" : "error cannot write screenshot";
  }
  return {"error unknown command: ", name};
}

auto Harness::serve() -> void {
  std::string line;
  while(std::getline(std::cin, line)) {
    auto words = nall::split(string{line.c_str()}.strip(), " ");
    std::erase_if(words, [](const string& word) { return !word; });
    if(words.empty()) continue;
    if(words[0] == "quit") break;
    fprintf(replies, "%s\n", execute(words).data());
    fflush(replies);
  }
}

}

auto nall::main(Arguments arguments) -> void {
  ares::Memory::FixedAllocator::get();

  string sramPath;
  arguments.take("--sram", sramPath);
  bool homebrew = arguments.take("--homebrew");
  bool skipsSpins = !arguments.take("--no-spin-skip");
  if(arguments.size() != 1) {
    print(stderr, "usage: n64-harness [--sram FILE] [--homebrew] [--no-spin-skip] ROM\n");
    exit(EXIT_FAILURE);
  }

  // Keep mia away from the user's ares save directory.
  mia::setSaveLocation([] { return string{"/nonexistent/"}; });

  static Harness harness;
  ares::platform = &harness;
  // The replies keep the original stdout; everything else printed there,
  // such as ares's notices, goes to stderr.
  harness.replies = fdopen(dup(STDOUT_FILENO), "w");
  if(!harness.replies || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
    print(stderr, "error: cannot separate the replies from stdout\n");
    exit(EXIT_FAILURE);
  }
  if(!harness.load(arguments.take(), sramPath, homebrew, skipsSpins)) exit(EXIT_FAILURE);
  harness.serve();
  harness.root->unload();
}
