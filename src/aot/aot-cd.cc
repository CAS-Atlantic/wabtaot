#include "../common.h" // for ReadFile, DataOrNull.

#include "../interp/binary-reader-interp.h"
#include "../binary-reader.h"
#include "../cast.h"
#include "../error-formatter.h"
#include "../feature.h"
#include "../interp/interp.h"
#include "../literal.h"
#include "../option-parser.h"
#include "../resolve-names.h"
#include "../stream.h"
#include "../validator.h"
#include "../wast-lexer.h"
#include "../wast-parser.h"

#include "aot-type-dictionary.h"
#include "aot-function-builder.h"
#include "trap-with.h"

#include "JitBuilder.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <math.h>
#include <string>
#include <dlfcn.h>
#include <time.h>

using namespace wabt;
using namespace wabt::interp;
/*
class WasmInterpHostImportDelegate : public HostImportDelegate {
 public:
  wabt::Result ImportFunc(interp::FuncImport* import,
                          interp::Func* func,
                          interp::FuncSignature* func_sig,
                          const ErrorCallback& callback) override {
    if (import->field_name!="") {
      cast<HostFunc>(func)->callback = PrintCallback;
      return wabt::Result::Ok;
    } else {
      
      return wabt::Result::Error;
    }
  }

  wabt::Result ImportTable(interp::TableImport* import,
                           interp::Table* table,
                           const ErrorCallback& callback) override {
    return wabt::Result::Error;
  }

  wabt::Result ImportMemory(interp::MemoryImport* import,
                            interp::Memory* memory,
                            const ErrorCallback& callback) override {
    return wabt::Result::Error;
  }

  wabt::Result ImportGlobal(interp::GlobalImport* import,
                            interp::Global* global,
                            const ErrorCallback& callback) override {
    return wabt::Result::Error;
  }

 private:
  static interp::Result PrintCallback(const HostFunc* func,
                                      const interp::FuncSignature* sig,
                                      Index num_args,
                                      TypedValue* args,
                                      Index num_results,
                                      TypedValue* out_results,
                                      void* user_data) {
    memset(out_results, 0, sizeof(TypedValue) * num_results);
    for (Index i = 0; i < num_results; ++i)
      out_results[i].type = sig->result_types[i];

    TypedValues vec_args(args, args + num_args);
    TypedValues vec_results(out_results, out_results + num_results);

    printf("called host ");
    
    return interp::Result::Ok;
  }

  void PrintError(const ErrorCallback& callback, const char* format, ...) {
    WABT_SNPRINTF_ALLOCA(buffer, length, format);
    callback(buffer);
  }
};*/

static Environment *envPointer;

extern int32_t internal_compileMethodBuilder(TR::MethodBuilder * methodBuilder, void ** entryPoint);

int numOfArgs;
char **args_arr;

static std::unique_ptr<FileStream> s_stdout_stream;
static std::unique_ptr<FileStream> s_log_stream;

static interp::Result PrintCallback(const HostFunc* func,
                                    const interp::FuncSignature* sig,
                                    const TypedValues& args,
                                    TypedValues& results) {
  printf("called host ");
  WriteCall(s_stdout_stream.get(), func->module_name, func->field_name, args,
            results, interp::Result::Ok);
  return interp::Result::Ok;
}

// from wasm-interp.cc in wasmjit-omr/src/tools.
static wabt::Result ReadModule(const char* module_filename,
                               Environment* env,
                               Errors* errors,
                               DefinedModule* out_module)
{
  wabt::Result result;
  std::vector<uint8_t> file_data;

  HostModule* host_module = env->AppendHostModule("host");
  host_module->on_unknown_func_export =
      [](Environment* env, HostModule* host_module, string_view name,
         Index sig_index) -> Index {

    if (name != "") {
      std::pair<HostFunc*, Index> pair =
        host_module->AppendFuncExport(name, sig_index, PrintCallback);
      return pair.second;
    }

    return kInvalidIndex;
    
  };

  HostModule *wasi = env->AppendHostModule("wasi_unstable");
  wasi->on_unknown_func_export =
      [](Environment* env, HostModule* host_module, string_view name,
         Index sig_index) -> Index {

    if (name != "") {
      std::pair<HostFunc*, Index> pair =
        host_module->AppendFuncExport(name, sig_index, PrintCallback);
      return pair.second;
    }

    return kInvalidIndex;
    
   };

  HostModule *envi = env->AppendHostModule("env");
  envi->on_unknown_func_export =
      [](Environment* env, HostModule* host_module, string_view name,
         Index sig_index) -> Index {

    if (name != "") {
      std::pair<HostFunc*, Index> pair =
        host_module->AppendFuncExport(name, sig_index, PrintCallback);
      return pair.second;
    }

    return kInvalidIndex;
    
  };

  envi->on_unknown_export =
      [](Environment* env, HostModule* module, string_view name, ExternalKind kind)
         -> Index {
    if (name != "") {
      
      switch(kind) {
          case ExternalKind::Memory: {
              auto pair = module->AppendMemoryExport(name, Limits(256, 256));
              return pair.second;
          }
          case ExternalKind::Table: {
              auto pair = module->AppendTableExport(name, Type::Funcref, Limits(6));
              return pair.second;
          }
      }
    }

    return kInvalidIndex;
    
  };

  // *out_module = nullptr;

  result = ReadFile(module_filename, &file_data);
  if (Succeeded(result)) {
    const bool kReadDebugNames = true;
    const bool kStopOnFirstError = true;
    const bool kFailOnCustomSectionError = true;

    Features s_features;
    ReadBinaryOptions options(s_features, s_log_stream.get(), kReadDebugNames,
                              kStopOnFirstError, kFailOnCustomSectionError);
    result = ReadBinaryInterp(env, file_data.data(), file_data.size(),
                              &options, errors, out_module);
/*
    if (Succeeded(result)) {
      if (s_verbose) {
        env->DisassembleModule(s_stdout_stream.get(), out_module);
      }
    }*/
  }
  return result;
}

void getCompiledFunction(const char *name, void (**fn)())
{
  *fn = reinterpret_cast<void(*)()>(getCodeEntry(const_cast<char*>(name)));
}

wabt::interp::Environment *getEnvironment()
{
  return envPointer;
}

wabt::Result compileAOT(interp::Environment& env, DefinedModule* module)
{
  using namespace wabt::aot;

  interp::Thread thread(&env);
  
  auto func_count = env.GetFuncCount();
  
  AOTManager aotManager;
  env.FillMemories();
  for(Index i = 0; i < func_count; ++i) {
    if(!env.GetFunc(i)->is_compiled) {
      auto* fn = cast<wabt::interp::DefinedFunc>(env.GetFunc(i));
      std::unique_ptr<AOTTypeDictionary> types(new (PERSISTENT_NEW) AOTTypeDictionary());
      //static AOTTypeDictionary types;
      std::string name = "f" + std::to_string(i) +"m"+module->name.substr(0,3);
      
      AOTFunctionBuilder* builder = new (PERSISTENT_NEW) AOTFunctionBuilder(&thread, fn,
							   std::move(name),
							   types.get(),
							   env, aotManager);

      std::unique_ptr<AOTFunctionBuilder> builder_ptr(builder);

      aotManager.push_back_FB(fn->offset, std::move(builder_ptr), std::move(types));
      
      env.GetFunc(i)->dbg_name_ = "f" + std::to_string(i) +"m"+module->name.substr(0,3);
      module->funcs.emplace_back(env.GetFunc(i));
      
    }else{
      aotManager.push_back_import(env.GetFunc(i)->dbg_name_,env.GetFunc(i));
        
      // for(Index j = 0;j<env.GetModuleCount();j++){
      //   for(auto exp:env.GetModule(j)->exports){
      //     if(!exp.name.compare(dynamic_cast<HostFunc*>(env.GetFunc(i))->field_name)){
      //       aotManager.push_back_import("f" + std::to_string(exp.index) +"m"+env.GetModule(j)->name.substr(0,3),env.GetFunc(i));
      //       env.GetFunc(i)->dbg_name_ = "f" + std::to_string(exp.index) +"m"+env.GetModule(j)->name.substr(0,3),env.GetFunc(i);
      //     }
      //   }
      // }
      
    }
    
  }

  aotManager.broadcastNames();
  aotManager.broadcastImports();
  module->compiled_functions.reserve(func_count);
  //std::fill(module->compiled_functions.begin(),module->compiled_functions.end(),nullptr);

  for(Index i = 0; i < func_count; ++i) {
    if(!env.GetFunc(i)->is_compiled) {
      auto* fn = cast<wabt::interp::DefinedFunc>(env.GetFunc(i));
      auto& builder = aotManager.getFB(fn->offset);
      void* function = nullptr;
      function = getCodeEntry(const_cast<char*>(fn->dbg_name_.c_str()));
      if(!function) {
        internal_compileMethodBuilder(&builder, &function);
        storeCodeEntry((char *)fn->dbg_name_.c_str());
	function = getCodeEntry(const_cast<char*>(fn->dbg_name_.c_str()));
      }
      module->compiled_functions.push_back(function);
      env.GetFunc(i)->is_compiled = true;
    }
  }
  return wabt::Result::Ok;
}

uint32_t printaa(int32_t a,int32_t b,int32_t c,int32_t d) { 
  uint32_t bufferLoc = *(uint32_t*)(envPointer->GetMems()[0]+b);
  char *buffer = envPointer->GetMems()[0]+bufferLoc;
  uint32_t buffsize = *(uint32_t*)(envPointer->GetMems()[0]+b+4);
  if(buffsize){
  std::cout<<std::string(buffer,buffsize);
  }
  return buffsize;
}

uint32_t gettimeod(int32_t a, int32_t b) {
  static int i = 0;
  uint32_t bufferLoc1 = *(uint32_t*)(envPointer->GetMems()[0]+a);
  time_t *bufferLoc = (time_t*)(envPointer->GetMems()[0]+bufferLoc1);
  suseconds_t *miliLoc = (suseconds_t*)(envPointer->GetMems()[0]+bufferLoc1+8);
  struct timeval tv{};
  gettimeofday(&tv,NULL);
  //*bufferLoc = tv.tv_sec;
  //*miliLoc = tv.tv_usec;
  *bufferLoc = -1 - (++i);
  *miliLoc = -1 - (++i);
  return 0;
}

int32_t print1(int32_t a,int32_t b){std::cout<<a<<","<<b<<"\n"; return 0;}
void print2(int32_t a,int32_t b){std::cout<<a+b<<"\n";}
int32_t seek(int32_t a,int64_t b,int32_t c,int32_t d) { std::cout<<a<<b<<c<<d<<"\n"; return 0;}
int32_t clos(int32_t a){ std::cout<<a; return 0; };
void clus(int32_t a){ std::cout<<a;};

int32_t args_get(int32_t argv,int32_t argv_buf) {
  uint32_t *bufferLoc = (uint32_t*)(envPointer->GetMems()[0]+argv);
  uint8_t *bufferLocsize = (uint8_t*)(envPointer->GetMems()[0]+argv_buf);
  *bufferLoc = argv_buf;
  if(numOfArgs>1)
    *(bufferLoc+1) = argv_buf+strlen(args_arr[1]);
  memcpy(bufferLocsize,args_arr[1],strlen(args_arr[1])+1);
  if(numOfArgs>1)
    memcpy(bufferLocsize+strlen(args_arr[1]),args_arr[2],strlen(args_arr[2])+1);
  return 0;
}
int32_t args_size_get(int32_t numOfArgs1, int32_t sizeOfArgs1){
  uint32_t *bufferLoc = (uint32_t*)(envPointer->GetMems()[0]+numOfArgs1);
  uint32_t *bufferLocsize = (uint32_t*)(envPointer->GetMems()[0]+sizeOfArgs1);
  *bufferLoc = numOfArgs;
  if(numOfArgs>1)
    *bufferLocsize = strlen(args_arr[2])+strlen(args_arr[1])+2;
  else
    *bufferLocsize = 0;
  return 0;
}


void funpr(uint64_t a) { std::cout<<((char*)(&a)); }

void relocateAOT(interp::Environment& env,DefinedModule *module)
{
  auto func_count = env.GetFuncCount();
  setCodeEntry("trapWith",reinterpret_cast<void*>(trapWith));
  double(*sqr)(double) = sqrt;
  setCodeEntry("sqrt",reinterpret_cast<void*>(sqr));
  double(*cpsign)(double,double) = copysign;
  setCodeEntry("copysign",reinterpret_cast<void*>(cpsign));
  setCodeEntry("sqrtf",reinterpret_cast<void*>(sqrtf));
  setCodeEntry("copysignf",reinterpret_cast<void*>(copysignf));
  setCodeEntry("CallIndi",reinterpret_cast<void*>(wabt::aot::AOTFunctionBuilder::CallIndirectHelper));
  setCodeEntry("GrowMem",reinterpret_cast<void*>(wabt::aot::AOTFunctionBuilder::GrowMemory));
  setCodeEntry("fd_write",reinterpret_cast<void*>(printaa));
  setCodeEntry("__lock",reinterpret_cast<void*>(1));
  setCodeEntry("__unlock",reinterpret_cast<void*>(1));
  setCodeEntry("emscripten_memcpy_big",reinterpret_cast<void*>(1));
  setCodeEntry("emscripten_resize_heap",reinterpret_cast<void*>(1));
  setCodeEntry("setTempRet0",reinterpret_cast<void*>(1));
  setCodeEntry("memory",reinterpret_cast<void*>(1));
  setCodeEntry("table",reinterpret_cast<void*>(1));
  setCodeEntry("emscript",reinterpret_cast<void*>(clus));
  setCodeEntry("setTempR",reinterpret_cast<void*>(1));
  setCodeEntry("Popcount",reinterpret_cast<void*>(static_cast<int(*)(unsigned)>(wabt::Popcount)));
  setCodeEntry("Popcountll",reinterpret_cast<void*>(static_cast<int(*)(unsigned long long)>(wabt::Popcount)));
  setCodeEntry("args_siz",reinterpret_cast<void*>(args_size_get));
  setCodeEntry("args_get",reinterpret_cast<void*>(args_get));
  setCodeEntry("proc_exi",reinterpret_cast<void*>(clus));
  setCodeEntry("fd_seek",reinterpret_cast<void*>(seek));
  setCodeEntry("fd_close",reinterpret_cast<void*>(clos));
  setCodeEntry("funpr",reinterpret_cast<void*>(funpr));
  setCodeEntry("gettimeo",reinterpret_cast<void*>(gettimeod));
  // for(Index i = 0; i < func_count; ++i) {
  //   if(env.GetFunc(i)->is_host) {
  //     setCodeEntry()
  //   }
  // }
  //setCodeEntry(const_cast<char*>(env.GetFunc(0)->dbg_name_.data()),reinterpret_cast<void*>(print));
  //setCodeEntry(const_cast<char*>(env.GetFunc(1)->dbg_name_.data()),reinterpret_cast<void*>(print1));
  //setCodeEntry("print1",reinterpret_cast<void*>(print1));
  /*for(int i=0;i<env.GetFuncCount();i++){
    interp::Func *func = env.GetFunc(i);
    if(func->is_host){
      void *handle = dlopen("libc.so.6",RTLD_LAZY);
      if(!handle){
        std::cerr<<"Cannot open libc!"<<"\n";
        exit(-1);
      }
      void *cfunc = dlsym(handle,func->dbg_name_.c_str());
      if(!cfunc){
        std::cerr<<"Cannot find "<<func->dbg_name_<<"\n";
        exit(-1);
      }
      setCodeEntry(const_cast<char*>(func->dbg_name_.data()),cfunc);
    }
    }*/
  Value *globals = new Value[env.GetGlobalCount()]();
  std::vector<std::string> global_names;
  for(int i=0;i<env.GetGlobalCount();i++) {
    globals[i] = env.GetGlobal(i)->typed_value.value;
    char global_name[6];
    sprintf(global_name,"g%d",i);
    global_names.emplace_back(global_name);
    setCodeEntry(const_cast<char*>(global_names.back().data()),reinterpret_cast<void*>(globals+i));
  }
  env.FillMemories();
  char memory_name[6];
  for(int i=0;i<env.GetMemoryCount();i++) {
    
    sprintf(memory_name,"m%d",i);
    //global_names.emplace_back(global_name);
    setCodeEntry(const_cast<char*>(memory_name),reinterpret_cast<void*>(env.GetMems()+i));
  }
  setCodeEntry(const_cast<char*>("Params"), reinterpret_cast<void*>(&env.indirectCallParams));
  // uint16_t compiled_function_index = 0;
  for(Index i = 0; i < module->compiled_functions.size(); ++i) {
    // if(!env.GetFunc(i)->is_host) {
    if(module->compiled_functions[i]){
      auto* fn = static_cast<DefinedFunc*>(module->funcs[i]);
      relocateCodeEntry(const_cast<char *>(fn->dbg_name_.c_str()));
      // compiled_function_index++;
    }
    // }
  }
}

void runExports(interp::Environment& env,DefinedModule *module)
{
  
  for(auto exported:module->exports){
    if(exported.kind != ExternalKind::Func) { continue;}
    std::string index = std::to_string(exported.index);
    void *fn = nullptr;
        if(exported.name != "_start") continue;
    for(uint32_t i = 0;i<module->funcs.size();i++){
      if(!index.compare(module->funcs[i]->dbg_name_.substr(1,index.size()))){
        fn = module->compiled_functions[i];
        break;
      }
    }
    // void *fun = module->compiled_functions[exported.index];
    if(env.GetFuncSignature(env.GetFunc(exported.index)->sig_index)->result_types.size()){
      if(env.GetFuncSignature(env.GetFunc(exported.index)->sig_index)->result_types.front() == Type::F32) {
        float a = reinterpret_cast<float(*)()>(fn)();
        std::cout<<"Export "<<exported.name<<" : "<<a<<"\n";
        }
      else if(env.GetFuncSignature(env.GetFunc(exported.index)->sig_index)->result_types.front() == Type::F64) {
        double a = reinterpret_cast<double(*)()>(fn)();
        std::cout<<"Export "<<exported.name<<" : "<<a<<"\n";
        }
      else {	
      uint64_t a = reinterpret_cast<uint64_t(*)()>(fn)();
      std::cout<<"Export "<<exported.name<<" : "<<a<<"\n";
      }
    }else{
      reinterpret_cast<void(*)()>(fn)();
    }
  }
}

void registerModules(const char* module_filename, Environment* env){
  std::string module_name(module_filename);
  // env->AppendHostModule(module.substr(0,module.find(".")))->import_delegate.reset(new WasmInterpHostImportDelegate());
  DefinedModule* module = new DefinedModule();
  module->name = module_name.substr(module_name.find_last_of('/')+1,module_name.find_last_of('.')-module_name.find_last_of('/')-1);
  //module->name = module_name.substr(0,module_name.find_last_of('.'));
  //module->name = module_name;//.substr(module_name.find_last_of('/')+1,3);
  env->AppendDefModule(module);
}

int main(int argc, char** argv) {
  if(argc < 2) {
    std::cout << "usage: wabtaot <filename>\n";
    return -1;
  }

  numOfArgs = argc-1;
  args_arr = argv;
  
  Environment env;
  s_stdout_stream = FileStream::CreateStdout();
  s_log_stream = FileStream::CreateStdout();
//  for(uint32_t i = 1;i<argc;i++) {
    registerModules(argv[1],&env);
//  }
  envPointer = &env;
  for(uint32_t i = 1;i<2;i++) {
    // const char* ffi = strrchr(argv[i],'/');
    // printf("%s\n",ffi);
    // char* src_filename;
    // if(ffi){
    //   src_filename = reinterpret_cast<char*>(calloc(strlen(ffi),1));
    //   strcpy(src_filename,ffi+1);
    // } else{
    char * src_filename = argv[i];
    // }

    DefinedModule* module = nullptr; //new DefinedModule();
    //ErrorHandlerFile error_handler(Location::Type::Binary);
    Errors errors;
    module = dynamic_cast<DefinedModule*>(env.GetModule(i-1));
    wabt::Result result = ReadModule(src_filename, &env, &errors, module);

    if(Succeeded(result)) {
      compileAOT(env, module);
    }else{
      std::cout<<"read failure\n";
    }
  }

  for(uint32_t i = 1;i<2;i++) {
    relocateAOT(env, dynamic_cast<DefinedModule*>(env.GetModule(i-1)));
  }
  for(uint32_t i = 1;i<2;i++) {
    runExports(env,dynamic_cast<DefinedModule*>(env.GetModule(i-1)));
  }
  //runExports(env, module);
}
