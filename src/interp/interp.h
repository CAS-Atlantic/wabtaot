/*
 * Copyright 2016 WebAssembly Community Group participants
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WABT_INTERP_H_
#define WABT_INTERP_H_

#include <stdint.h>
#include <elf.h>
#include <stdio.h>

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_map>
#include <sys/mman.h>

#include "src/jit/environment.h"
#include "src/binding-hash.h"
#include "src/common.h"
#include "src/opcode.h"
#include "src/stream.h"

namespace wabt {

namespace jit {
class FunctionBuilder;
}

namespace aot {
class AOTFunctionBuilder;  
}

namespace interp {
  
class ELFLoader
{
public:
  ELFLoader(char *elfFileName);

  ~ELFLoader();
  void *getTextSection();
  Elf64_Sym *getSymbolTable();
  unsigned int *getCustomSection();
  void printHeader();
  void printSymbolTable();
  
protected:
  typedef Elf64_Ehdr ELFEHeader;
  typedef Elf64_Shdr ELFSectionHeader;
  typedef Elf64_Phdr ELFProgramHeader;
  typedef Elf64_Addr ELFAddress;
  typedef Elf64_Sym  ELFSymbol;
  typedef Elf64_Rela ELFRela;
  typedef Elf64_Off  ELFOffset;
#define ELF_ST_INFO(bind, type) ELF64_ST_INFO(bind,type)
#define ELF_ST_VISIBILITY(visibility) ELF64_ST_VISIBILITY(visibility)
#define ELF_R_INFO(bind, type) ELF64_R_INFO(bind, type)
#define ELFClass ELFCLASS64;
#define BIT(x,n) (((x)>>(n))&1)

  char       *_elfFileName;
  FILE       *_elfFile;
  ELFEHeader *_header;
  
  ELFSectionHeader *_zeroSection;
  char              _zeroSectionName[1];
  ELFSectionHeader *_textSection;
  char              _textSectionName[6];
  ELFSectionHeader *_relaSection;
  char              _relaSectionName[11];
  ELFSectionHeader *_dynSymSection;
  char              _dynSymSectionName[8];
  ELFSectionHeader *_shStrTabSection;
  char              _shStrTabSectionName[10];
  ELFSectionHeader *_dynStrSection;
  char              _dynStrSectionName[8];
  ELFSectionHeader *_customSection;
  char              _customSectionName[8];

  void *_text;
  Elf64_Sym *_symtab;
  char *_dynstr;
  Elf64_Rela *_rela;
  unsigned int *_custom;

  void initialize();
  void loadTextSection();
  void loadSymTab();
  void loadDynStr();
  void loadRela();
  void loadCustom();
  char *typeString(ELFSectionHeader *);
  char *flagString(ELFSectionHeader *);
  char *symTypeString(Elf64_Sym);
  char *symBindString(Elf64_Sym);
  char *symVisString(Elf64_Sym);
  char *symNdxString(Elf64_Sym);
  char *symNameString(Elf64_Sym);
  
}; //class ELFLoader


#define FOREACH_INTERP_RESULT(V)                                            \
  V(Ok, "ok")                                                               \
  /* returned from the top-most function */                                 \
  V(Returned, "returned")                                                   \
  /* memory access is out of bounds */                                      \
  V(TrapMemoryAccessOutOfBounds, "out of bounds memory access")             \
  /* atomic memory access is unaligned  */                                  \
  V(TrapAtomicMemoryAccessUnaligned, "atomic memory access is unaligned")   \
  /* converting from float -> int would overflow int */                     \
  V(TrapIntegerOverflow, "integer overflow")                                \
  /* dividend is zero in integer divide */                                  \
  V(TrapIntegerDivideByZero, "integer divide by zero")                      \
  /* converting from float -> int where float is nan */                     \
  V(TrapInvalidConversionToInteger, "invalid conversion to integer")        \
  /* function table index is out of bounds */                               \
  V(TrapUndefinedTableIndex, "undefined table index")                       \
  /* function table element is uninitialized */                             \
  V(TrapUninitializedTableElement, "uninitialized table element")           \
  /* unreachable instruction executed */                                    \
  V(TrapUnreachable, "unreachable executed")                                \
  /* call indirect signature doesn't match function table signature */      \
  V(TrapIndirectCallSignatureMismatch, "indirect call signature mismatch")  \
  /* ran out of call stack frames (probably infinite recursion) */          \
  V(TrapCallStackExhausted, "call stack exhausted")                         \
  /* ran out of value stack space */                                        \
  V(TrapValueStackExhausted, "value stack exhausted")                       \
  /* we called a host function, but the return value didn't match the */    \
  /* expected type */                                                       \
  V(TrapHostResultTypeMismatch, "host result type mismatch")                \
  /* we called an import function, but it didn't complete succesfully */    \
  V(TrapHostTrapped, "host function trapped")                               \
  V(TrapFailedAOTLookup, "AOT lookup failed")                               \
  /* we attempted to JIT compile a function and failed */                   \
  V(TrapFailedJITCompilation, "failed JIT compilation")                     \
  /* the data segment has been dropped. */                                  \
  V(TrapDataSegmentDropped, "data segment dropped")                         \
  /* the element segment has been dropped. */                               \
  V(TrapElemSegmentDropped, "element segment dropped")                      \
  /* table access is out of bounds */                                       \
  V(TrapTableAccessOutOfBounds, "out of bounds table access")               \
  /* we attempted to call a function with the an argument list that doesn't \
   * match the function signature */                                        \
  V(ArgumentTypeMismatch, "argument type mismatch")                         \
  /* we tried to get an export by name that doesn't exist */                \
  V(UnknownExport, "unknown export")                                        \
  /* the expected export kind doesn't match. */                             \
  V(ExportKindMismatch, "export kind mismatch")

enum class Result : int32_t {
#define V(Name, str) Name,
  FOREACH_INTERP_RESULT(V)
#undef V
};

extern bool trapFlag;
extern Result trapResult;

typedef uint32_t IstreamOffset;
static const IstreamOffset kInvalidIstreamOffset = ~0;

struct FuncSignature {
  FuncSignature() = default;
  FuncSignature(std::vector<Type> param_types, std::vector<Type> result_types);
  FuncSignature(Index param_count,
                Type* param_types,
                Index result_count,
                Type* result_types);

  std::vector<Type> param_types;
  std::vector<Type> result_types;
};

struct Table {
  explicit Table(Type elem_type, const Limits& limits)
      : elem_type(elem_type),
        limits(limits),
        func_indexes(limits.initial, kInvalidIndex) {}

  Type elem_type;
  Limits limits;
  std::vector<Index> func_indexes;
};

struct Memory {
  Memory() = default;
  explicit Memory(const Limits& limits)
    : page_limits(limits){
	madvise(data.data(), 2368709120*sizeof(char), MADV_SEQUENTIAL);
	madvise(data.data(), 2368709120*sizeof(char), MADV_HUGEPAGE);  
	madvise(data.data(), 2368709120*sizeof(char), MADV_WILLNEED);
	
      }

  Limits page_limits;
  alignas(4096) std::array<char,2368709120> data;
};

struct DataSegment {
  DataSegment() = default;

  std::vector<char> data;
  bool dropped = false;
};

struct ElemSegment {
  ElemSegment() = default;

  std::vector<Index> elems;
  bool dropped = false;
};

// ValueTypeRep converts from one type to its representation on the
// stack. For example, float -> uint32_t. See Value below.
template <typename T>
struct ValueTypeRepT;

template <> struct ValueTypeRepT<int32_t> { typedef uint32_t type; };
template <> struct ValueTypeRepT<uint32_t> { typedef uint32_t type; };
template <> struct ValueTypeRepT<int64_t> { typedef uint64_t type; };
template <> struct ValueTypeRepT<uint64_t> { typedef uint64_t type; };
template <> struct ValueTypeRepT<float> { typedef uint32_t type; };
template <> struct ValueTypeRepT<double> { typedef uint64_t type; };
template <> struct ValueTypeRepT<v128> { typedef v128 type; };

template <typename T>
using ValueTypeRep = typename ValueTypeRepT<T>::type;

union Value {
  uint32_t i32;
  uint64_t i64;
  ValueTypeRep<float> f32_bits;
  ValueTypeRep<double> f64_bits;
  ValueTypeRep<v128> v128_bits;
};

struct TypedValue {
  TypedValue() {}
  explicit TypedValue(Type type) : type(type) {}
  TypedValue(Type type, const Value& value) : type(type), value(value) {}

  void SetZero() { ZeroMemory(value); }
  void set_i32(uint32_t x) { value.i32 = x; }
  void set_i64(uint64_t x) { value.i64 = x; }
  void set_f32(float x) { memcpy(&value.f32_bits, &x, sizeof(x)); }
  void set_f64(double x) { memcpy(&value.f64_bits, &x, sizeof(x)); }

  uint32_t get_i32() const { return value.i32; }
  uint64_t get_i64() const { return value.i64; }
  float get_f32() const {
    float x;
    memcpy(&x, &value.f32_bits, sizeof(x));
    return x;
  }
  double get_f64() const {
    double x;
    memcpy(&x, &value.f64_bits, sizeof(x));
    return x;
  }

  Type type;
  Value value;
};

typedef std::vector<TypedValue> TypedValues;

struct Global {
  Global() : mutable_(false), import_index(kInvalidIndex) {}
  Global(const TypedValue& typed_value, bool mutable_)
      : typed_value(typed_value), mutable_(mutable_) {}

  TypedValue typed_value;
  bool mutable_;
  Index import_index; /* or INVALID_INDEX if not imported */
};

struct Import {
  explicit Import(ExternalKind kind) : kind(kind) {}
  Import(ExternalKind kind, string_view module_name, string_view field_name)
      : kind(kind),
        module_name(module_name.to_string()),
        field_name(field_name.to_string()) {}

  ExternalKind kind;
  std::string module_name;
  std::string field_name;
};

struct FuncImport : Import {
  FuncImport() : Import(ExternalKind::Func) {}
  FuncImport(string_view module_name, string_view field_name)
      : Import(ExternalKind::Func, module_name, field_name) {}

  Index sig_index = kInvalidIndex;
};

struct TableImport : Import {
  TableImport() : Import(ExternalKind::Table) {}
  TableImport(string_view module_name, string_view field_name)
      : Import(ExternalKind::Table, module_name, field_name) {}

  Limits limits;
};

struct MemoryImport : Import {
  MemoryImport() : Import(ExternalKind::Memory) {}
  MemoryImport(string_view module_name, string_view field_name)
      : Import(ExternalKind::Memory, module_name, field_name) {}

  Limits limits;
};

struct GlobalImport : Import {
  GlobalImport() : Import(ExternalKind::Global) {}
  GlobalImport(string_view module_name, string_view field_name)
      : Import(ExternalKind::Global, module_name, field_name) {}

  Type type = Type::Void;
  bool mutable_ = false;
};

struct EventImport : Import {
  EventImport() : Import(ExternalKind::Event) {}
  EventImport(string_view module_name, string_view field_name)
      : Import(ExternalKind::Event, module_name, field_name) {}
};

struct Func;

struct Func {
  WABT_DISALLOW_COPY_AND_ASSIGN(Func);
  Func(Index sig_index, bool is_host)
      : sig_index(sig_index), is_host(is_host),offset(kInvalidIstreamOffset) {}
  virtual ~Func() {}

  Index sig_index;
  bool is_host;
  bool is_compiled = false;
  std::string dbg_name_ = "???";
  IstreamOffset offset;
};

struct DefinedFunc : Func {
  DefinedFunc(Index sig_index)
      : Func(sig_index, false),
        
        local_decl_count(0),
        local_count(0) {}

  static bool classof(const Func* func) { //return !func->is_host; 
    return true;
  }

  
  bool has_dbg_name_ = false;

  
  Index local_decl_count;
  Index local_count;
  
  // first the parameter types, and then the local types.
  // the number of local types is given by local_count.
  std::vector<Type> param_and_local_types;
};

struct HostFunc : Func {
  using Callback = std::function<Result(const HostFunc*,
                                        const FuncSignature*,
                                        const TypedValues& args,
                                        TypedValues& results)>;

  HostFunc(string_view module_name,
           string_view field_name,
           Index sig_index,
           Callback callback)
      : Func(sig_index, true),
        module_name(module_name.to_string()),
        field_name(field_name.to_string()),
      // {
  //  }
      
        callback(callback) { is_compiled = true; }

  static bool classof(const Func* func) { return func->is_host; }

  std::string module_name;
  std::string field_name;
  Callback callback;
};

struct Export {
  Export(string_view name, ExternalKind kind, Index index)
      : name(name.to_string()), kind(kind), index(index) {}

  std::string name;
  ExternalKind kind;
  Index index;
};

class Environment;
struct DefinedModule;
struct HostModule;

struct Module {
  WABT_DISALLOW_COPY_AND_ASSIGN(Module);
  explicit Module(bool is_host);
  Module(string_view name, bool is_host);
  virtual ~Module() = default;

  // Function exports are special-cased to allow for overloading functions by
  // name.
  Export* GetFuncExport(Environment*, string_view name, Index sig_index);
  Export* GetExport(string_view name);
  virtual Index OnUnknownFuncExport(string_view name, Index sig_index) = 0;

  // Returns export index.
  Index AppendExport(ExternalKind kind, Index item_index, string_view name);

  std::string name;
  std::vector<Export> exports;
  BindingHash export_bindings;
  Index memory_index; /* kInvalidIndex if not defined */
  Index table_index;  /* kInvalidIndex if not defined */
  bool is_host;
};

struct DefinedModule : Module {
  DefinedModule();
  static bool classof(const Module* module) { return !module->is_host; }

  Index OnUnknownFuncExport(string_view name, Index sig_index) override {
    return kInvalidIndex;
  }

  std::vector<FuncImport> func_imports;
  std::vector<TableImport> table_imports;
  std::vector<MemoryImport> memory_imports;
  std::vector<GlobalImport> global_imports;
  std::vector<EventImport> event_imports;
  Index start_func_index; /* kInvalidIndex if not defined */
  IstreamOffset istream_start;
  IstreamOffset istream_end;
  std::vector<void*> compiled_functions;
  std::vector<Func*> funcs;
};

struct HostModule : Module {
  HostModule(Environment* env, string_view name);
  static bool classof(const Module* module) { return module->is_host; }

  Index OnUnknownFuncExport(string_view name, Index sig_index) override;
  Export* GetExport(string_view, ExternalKind);
  Index OnUnknownExport(string_view, ExternalKind);

  std::pair<HostFunc*, Index> AppendFuncExport(string_view name,
                                               const FuncSignature&,
                                               HostFunc::Callback);
  std::pair<HostFunc*, Index> AppendFuncExport(string_view name,
                                               Index sig_index,
                                               HostFunc::Callback);
  std::pair<Table*, Index> AppendTableExport(string_view name,
                                             Type elem_type,
                                             const Limits&);
  std::pair<Memory*, Index> AppendMemoryExport(string_view name, const Limits&);
  std::pair<Global*, Index> AppendGlobalExport(string_view name,
                                               Type,
                                               bool mutable_);

  // Convenience functions.
  std::pair<Global*, Index> AppendGlobalExport(string_view name,
                                               bool mutable_,
                                               uint32_t);
  std::pair<Global*, Index> AppendGlobalExport(string_view name,
                                               bool mutable_,
                                               uint64_t);
  std::pair<Global*, Index> AppendGlobalExport(string_view name,
                                               bool mutable_,
                                               float);
  std::pair<Global*, Index> AppendGlobalExport(string_view name,
                                               bool mutable_,
                                               double);

  // Should return an Export index if a new function was created via
  // AppendFuncExport, or kInvalidIndex if no function was created.
  std::function<
      Index(Environment*, HostModule*, string_view name, Index sig_index)>
      on_unknown_func_export;
  std::function<
      Index(Environment*, HostModule*, string_view name, ExternalKind)>
      on_unknown_export;

 private:
  Environment* env_;
};

class Thread;
class Environment {
 public:
  // Used to track and reset the state of the environment.
  struct MarkPoint {
    size_t modules_size = 0;
    size_t sigs_size = 0;
    size_t funcs_size = 0;
    size_t memories_size = 0;
    size_t tables_size = 0;
    size_t globals_size = 0;
    size_t data_segments_size = 0;
    size_t elem_segments_size = 0;
    size_t istream_size = 0;
  };

  bool enable_jit = true;
  bool trap_on_failed_comp = false;
  bool enable_load_from_dlib = false;
  bool enable_load_thunk = false;
  char *infile = nullptr;
  uint32_t jit_threshold = 1;

  Environment();
  ~Environment();

  OutputBuffer& istream() { return *istream_; }
  void SetIstream(std::unique_ptr<OutputBuffer> istream) {
    istream_ = std::move(istream);
  }
  
  std::unique_ptr<OutputBuffer> ReleaseIstream() { return std::move(istream_); }

  Index GetFuncSignatureCount() const { return sigs_.size(); }
  Index GetFuncCount() const { return funcs_.size(); }
  Index GetGlobalCount() const { return globals_.size(); }
  Index GetMemoryCount() const { return memories_.size(); }
  Index GetTableCount() const { return tables_.size(); }
  Index GetDataSegmentCount() const { return data_segments_.size(); }
  Index GetElemSegmentCount() const { return elem_segments_.size(); }
  Index GetModuleCount() const { return modules_.size(); }

  Index GetLastModuleIndex() const {
    return modules_.empty() ? kInvalidIndex : modules_.size() - 1;
  }
  Index FindModuleIndex(string_view name) const;

  FuncSignature* GetFuncSignature(Index index) { return &sigs_[index]; }
  Func* GetFunc(Index index) {
    assert(index < funcs_.size());
    return funcs_[index].get();
  }
  Global* GetGlobal(Index index) {
    assert(index < globals_.size());
    return &globals_[index];
  }
  Memory* GetMemory(Index index) {
    assert(index < memories_.size());
    return &memories_[index];
  }
  Table* GetTable(Index index) {
    assert(index < tables_.size());
    return &tables_[index];
  }
  DataSegment* GetDataSegment(Index index) {
    assert(index < data_segments_.size());
    return &data_segments_[index];
  }
  ElemSegment* GetElemSegment(Index index) {
    assert(index < elem_segments_.size());
    return &elem_segments_[index];
  }
  Module* GetModule(Index index) {
    assert(index < modules_.size());
    return modules_[index].get();
  }

  Module* GetLastModule() {
    return modules_.empty() ? nullptr : modules_.back().get();
  }
  Module* FindModule(string_view name);
  Module* FindRegisteredModule(string_view name);

  template <typename... Args>
  FuncSignature* EmplaceBackFuncSignature(Args&&... args) {
    sigs_.emplace_back(std::forward<Args>(args)...);
    return &sigs_.back();
  }

  template <typename... Args>
  Func* EmplaceBackFunc(Args&&... args) {
    funcs_.emplace_back(std::forward<Args>(args)...);
    return funcs_.back().get();
  }

  void AddJitMetadata(Func* fn) {
    assert(fn->offset != kInvalidIstreamOffset);
    this->jit_meta_.insert({ fn->offset, JitMeta(fn) });
  }

  template <typename... Args>
  Global* EmplaceBackGlobal(Args&&... args) {
    globals_.emplace_back(std::forward<Args>(args)...);
    return &globals_.back();
  }

  template <typename... Args>
  Table* EmplaceBackTable(Args&&... args) {
    tables_.emplace_back(std::forward<Args>(args)...);
    return &tables_.back();
  }

  template <typename... Args>
  Memory* EmplaceBackMemory(Args&&... args) {
    memories_.emplace_back(std::forward<Args>(args)...);
    return &memories_.back();
  }

  template <typename... Args>
  DataSegment* EmplaceBackDataSegment(Args&&... args) {
    data_segments_.emplace_back(std::forward<Args>(args)...);
    return &data_segments_.back();
  }

  template <typename... Args>
  ElemSegment* EmplaceBackElemSegment(Args&&... args) {
    elem_segments_.emplace_back(std::forward<Args>(args)...);
    return &elem_segments_.back();
  }

  template <typename... Args>
  Module* EmplaceBackModule(Args&&... args) {
    modules_.emplace_back(std::forward<Args>(args)...);
    return modules_.back().get();
  }

  template <typename... Args>
  void EmplaceModuleBinding(Args&&... args) {
    module_bindings_.emplace(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void EmplaceRegisteredModuleBinding(Args&&... args) {
    registered_module_bindings_.emplace(std::forward<Args>(args)...);
  }

  uint64_t memoriesLoc(){ return reinterpret_cast<uint64_t>(memories_[0].data.data()); }
  char **GetMems() {return mems;}

  HostModule* AppendHostModule(string_view name);
  void AppendDefModule(DefinedModule*);

  bool FuncSignaturesAreEqual(Index sig_index_0, Index sig_index_1) const;

  MarkPoint Mark();
  void ResetToMarkPoint(const MarkPoint&);

  void Disassemble(Stream* stream, IstreamOffset from, IstreamOffset to);
  void DisassembleModule(Stream* stream, Module*);
  void LoadDLib(char *filename);
  void FillMemories();
  void FillTables();
  uint64_t *indirectCallParams = new uint64_t[8]();

 private:
  friend class Thread;
  friend class wabt::jit::FunctionBuilder;
  friend class wabt::aot::AOTFunctionBuilder;
  using JITedFunction = wabt::interp::Result (*)();
  using AOTedFunction = uint64_t (*)();

  struct JitMeta {
    Func* wasm_fn;
    uint32_t num_calls = 0;

    bool tried_jit = false;
    JITedFunction jit_fn = nullptr;

    JitMeta(Func* wasm_fn) : wasm_fn(wasm_fn) {
      //wasm_fn->dbg_name_= "func_" + std::to_string(numOfFunction);
      // wasm_fn->dbg_name_= "f" + wasm_fn-> +"m"+modules_[0]->name.substr(0,3);
      numOfFunction++;
    }
    private:
    static int numOfFunction;
  };

  bool TryJit(Thread* t, IstreamOffset offset, JITedFunction* fn);
  bool TryJit(Thread* t, IstreamOffset offset, JITedFunction* fn,DefinedFunc *&);

  std::vector<std::unique_ptr<Module>> modules_;
  std::vector<FuncSignature> sigs_;
  std::vector<std::unique_ptr<Func>> funcs_;
  std::vector<Memory> memories_;
  std::vector<Table> tables_;
  std::vector<Global> globals_;
  std::vector<DataSegment> data_segments_;
  std::vector<ElemSegment> elem_segments_;
  std::unique_ptr<OutputBuffer> istream_;
  BindingHash module_bindings_;
  BindingHash registered_module_bindings_;

  jit::JitEnvironment jit_env_;
  std::unordered_map<IstreamOffset, JitMeta> jit_meta_;
  ELFLoader *elfLoader = nullptr;
  char **mems = nullptr;
  Func **tabs = nullptr;
};


struct ThreadOffset;
 
struct CallFrame {
  CallFrame() : pc(0), is_jit(false), is_jit_compiling(false) {}
  CallFrame(IstreamOffset pc, bool is_jit, bool is_jit_compiling = false)
    : pc(pc), is_jit(is_jit), is_jit_compiling(is_jit_compiling) {}

  IstreamOffset pc;
  bool is_jit;
  bool is_jit_compiling;
};

class Thread {
 public:
  struct Options {
    static const uint32_t kDefaultValueStackSize = 512 * 1024 / sizeof(Value);
    static const uint32_t kDefaultCallStackSize = 64 * 1024;

    explicit Options(uint32_t value_stack_size = kDefaultValueStackSize,
                     uint32_t call_stack_size = kDefaultCallStackSize);

    uint32_t value_stack_size;
    uint32_t call_stack_size;
  };

  explicit Thread(Environment*, const Options& = Options());

  Environment* env() { return env_; }

  void set_pc(IstreamOffset offset) { pc_ = offset; }
  IstreamOffset pc() const { return pc_; }

  void Reset();
  Index NumValues() const { return value_stack_top_; }
  Result Push(Value) WABT_WARN_UNUSED;
  Value Pop();
  Value ValueAt(Index at) const;

  void Trace(Stream*);
  Result Run(int num_instructions = 1);
  Result CallThunk(Environment::JITedFunction,Func*);
  Result CallHost(HostFunc*);

 private:
  friend class wabt::jit::FunctionBuilder;
  friend class wabt::aot::AOTFunctionBuilder;
  friend class ThreadOffset;
  friend class Executor;
  
  const uint8_t* GetIstream() const { return env_->istream_->data.data(); }

  Memory* ReadMemory(const uint8_t** pc);
  template <typename MemType>
  Result GetAccessAddress(const uint8_t** pc, void** out_address);
  template <typename MemType>
  Result GetAtomicAccessAddress(const uint8_t** pc, void** out_address);

  Table* ReadTable(const uint8_t** pc);

  DataSegment* ReadDataSegment(const uint8_t** pc);
  ElemSegment* ReadElemSegment(const uint8_t** pc);

  Value& Top();
  Value& Pick(Index depth);

  // Push/Pop values with conversions, e.g. Push<float> will convert to the
  // ValueTypeRep (uint32_t) and push that. Similarly, Pop<float> will pop the
  // value and convert to float.
  template <typename T>
  Result Push(T) WABT_WARN_UNUSED;
  template <typename T>
  T Pop();

  // Push/Pop values without conversions, e.g. PushRep<float> will take a
  // uint32_t argument which is the integer representation of that float value.
  // Similarly, PopRep<float> will not convert the value to a float.
  template <typename T>
  Result PushRep(ValueTypeRep<T>) WABT_WARN_UNUSED;
  template <typename T>
  ValueTypeRep<T> PopRep();

  void DropKeep(uint32_t drop_count, uint32_t keep_count);

  Result PushCall(const uint8_t* pc, bool is_jit = false) WABT_WARN_UNUSED;
  IstreamOffset PopCall();

  template <typename R, typename T> using UnopFunc      = R(T);
  template <typename R, typename T> using UnopTrapFunc  = Result(T, R*);
  template <typename R, typename T> using BinopFunc     = R(T, T);
  template <typename R, typename T> using BinopTrapFunc = Result(T, T, R*);

  template <typename MemType, typename ResultType = MemType>
  Result Load(const uint8_t** pc) WABT_WARN_UNUSED;
  template <typename MemType, typename ResultType = MemType>
  Result Store(const uint8_t** pc) WABT_WARN_UNUSED;
  template <typename MemType, typename ResultType = MemType>
  Result AtomicLoad(const uint8_t** pc) WABT_WARN_UNUSED;
  template <typename MemType, typename ResultType = MemType>
  Result AtomicStore(const uint8_t** pc) WABT_WARN_UNUSED;
  template <typename MemType, typename ResultType = MemType>
  Result AtomicRmw(BinopFunc<ResultType, ResultType>,
                   const uint8_t** pc) WABT_WARN_UNUSED;
  template <typename MemType, typename ResultType = MemType>
  Result AtomicRmwCmpxchg(const uint8_t** pc) WABT_WARN_UNUSED;

  Result MemoryInit(const uint8_t** pc) WABT_WARN_UNUSED;
  Result DataDrop(const uint8_t** pc) WABT_WARN_UNUSED;
  Result MemoryCopy(const uint8_t** pc) WABT_WARN_UNUSED;
  Result MemoryFill(const uint8_t** pc) WABT_WARN_UNUSED;
  Result TableInit(const uint8_t** pc) WABT_WARN_UNUSED;
  Result ElemDrop(const uint8_t** pc) WABT_WARN_UNUSED;
  Result TableCopy(const uint8_t** pc) WABT_WARN_UNUSED;

  template <typename R, typename T = R>
  Result Unop(UnopFunc<R, T> func) WABT_WARN_UNUSED;
  template <typename R, typename T = R>
  Result UnopTrap(UnopTrapFunc<R, T> func) WABT_WARN_UNUSED;

  template <typename T, typename L, typename R, typename P = R>
  Result SimdUnop(UnopFunc<R, P> func) WABT_WARN_UNUSED;

  template <typename R, typename T = R>
  Result Binop(BinopFunc<R, T> func) WABT_WARN_UNUSED;
  template <typename R, typename T = R>
  Result BinopTrap(BinopTrapFunc<R, T> func) WABT_WARN_UNUSED;

  template <typename T, typename L, typename R, typename P = R>
  Result SimdBinop(BinopFunc<R, P> func) WABT_WARN_UNUSED;

  template <typename T, typename L, typename R, typename P = R>
  Result SimdRelBinop(BinopFunc<R, P> func) WABT_WARN_UNUSED;

  Environment* env_ = nullptr;
  std::vector<Value> value_stack_;
  //std::vector<IstreamOffset> call_stack_;
  Value *vs_array_;
  Value *vs_top_;
  std::vector<CallFrame> call_stack_;

  uint32_t value_stack_top_ = 0;
  uint32_t call_stack_top_ = 0;
  uint32_t last_jit_frame_ = 0;
  IstreamOffset pc_ = 0;
  bool in_jit_ = false;
};

struct ThreadOffset {
  static const std::size_t so = offsetof(Thread,vs_array_);
  static const std::size_t to = offsetof(Thread,vs_top_);
};

struct ExecResult {
  ExecResult() = default;
  explicit ExecResult(Result result) : result(result) {}
  ExecResult(Result result, const TypedValues& values)
      : result(result), values(values) {}

  void PrintCallStack(Stream*, Environment*);

  Result result = Result::Ok;
  TypedValues values;
  std::vector<CallFrame> call_stack;
};

class Executor {
 public:
  explicit Executor(Environment*,
                    Stream* trace_stream = nullptr,
                    const Thread::Options& options = Thread::Options());

  ExecResult RunFunction(Index func_index, const TypedValues& args);
  ExecResult RunStartFunction(DefinedModule* module);
  ExecResult RunExport(const Export*, const TypedValues& args);
  ExecResult RunExportByName(Module* module,
                             string_view name,
                             const TypedValues& args);

 private:
  Result RunDefinedFunction(IstreamOffset function_offset);
  Result PushArgs(const FuncSignature*, const TypedValues& args);
  void CopyResults(const FuncSignature*, TypedValues* out_results);

  Environment* env_ = nullptr;
  Stream* trace_stream_ = nullptr;
  Thread thread_;
};

bool IsCanonicalNan(uint32_t f32_bits);
bool IsCanonicalNan(uint64_t f64_bits);
bool IsArithmeticNan(uint32_t f32_bits);
bool IsArithmeticNan(uint64_t f64_bits);

std::string TypedValueToString(const TypedValue&);
const char* ResultToString(Result);

bool ClampToBounds(uint32_t start, uint32_t* length, uint32_t max);

void WriteTypedValue(Stream* stream, const TypedValue&);
void WriteTypedValues(Stream* stream, const TypedValues&);
void WriteResult(Stream* stream, const char* desc, Result);
void WriteCall(Stream* stream,
               string_view module_name,
               string_view func_name,
               const TypedValues& args,
               const TypedValues& results,
               Result);

}  // namespace interp
}  // namespace wabt

#endif /* WABT_INTERP_H_ */
