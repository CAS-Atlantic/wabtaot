/*
 * Copyright 2017 wasmjit-omr project participants
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

//#include "../cast.h"
//#include "../interp.h"
#include <cstddef>
#include "wabtjit.h"
#include "type-dictionary.h"
#include "function-builder.h"
//#include "infra/Assert.hpp"
#include <dlfcn.h>


//#include "JitBuilder.hpp"
extern int32_t internal_compileMethodBuilder(TR::MethodBuilder * methodBuilder, void ** entryPoint);

namespace wabt{
namespace jit {

JITedFunction compile(interp::Thread* thread, interp::Func* fn) {
  TypeDictionary types;
  FunctionBuilder builder(thread, dynamic_cast<interp::DefinedFunc*>(fn), &types);
  void* function = nullptr;
  
  if (internal_compileMethodBuilder(&builder, &function) == 0) {
    return reinterpret_cast<JITedFunction>(function);
  } else {
    return nullptr;
  }
}

JITedFunction loadCompiled(interp::Thread* thread, interp::Func* fn,
			   interp::Environment& env) {
  /*TypeDictionary types;
  //unsigned int numCalleeParams = env.GetFuncSignature(fn->sig_index)->param_types.size();
  unsigned int numCalleeParams = 1;
  TR::IlType** tv = new TR::IlType*[numCalleeParams];
  /*for(int i=0;i<numCalleeParams;i++) {
    tv[i] = TypeFieldType(env.GetFuncSignature(fn->sig_index)->param_types[i]);
    }*/
  /* tv[0] = types.threadPtr;
  FunctionThunkBuilder builder(&types,fn->dbg_name_.c_str(),functionReturnType(fn,env),
		       numCalleeParams,tv);
		       
  uint8_t* function = nullptr;
  
  if (compileMethodBuilder(&builder, &function) == 0) {
    return reinterpret_cast<JITedFunction>(function);
  } else {
    return nullptr;
    }*/
  void *handle = dlopen(env.infile,RTLD_LAZY);
  if(!handle) {
    return nullptr;
  }
  void *funct = dlsym(handle,fn->dbg_name_.c_str());
  return reinterpret_cast<JITedFunction>(funct);
}
/*
JITedFunction loadThunk(interp::Thread* thread, interp::Func* fn,
			interp::Environment& env) {
  AOTTypeDictionary types;
  unsigned int numCalleeParams = env.GetFuncSignature(fn->sig_index)->param_types.size();
  unsigned int memglcount = 0;
  if(env.GetMemoryCount()>0){
    numCalleeParams++;
    memglcount++;
  }
  if(env.GetGlobalCount()>0){
    numCalleeParams++;
    memglcount++;
  }
  OMR::JitBuilder::IlType** params = new OMR::JitBuilder::IlType*[numCalleeParams];
  if(env.GetMemoryCount()>0){
    //tv[0] = types.PointerTo(types.PointerTo(types.toIlType<int64_t>()));
    params[0] = types.toIlType<int64_t>();
    if(env.GetGlobalCount()>0)
      params[1] = types.toIlType<int64_t>();
  } else if (env.GetGlobalCount()>0){
    params[0] = types.toIlType<int64_t>();
  }
  for(int i=memglcount;i<numCalleeParams;i++) {
    params[i] = TypeFieldType(env.GetFuncSignature(fn->sig_index)->param_types[i-memglcount]);
  }
  
  OMR::JitBuilder::ThunkBuilder builder(&types,fn->dbg_name_.c_str(),
			   functionReturnType(dynamic_cast<interp::DefinedFunc*>(fn),env,types),numCalleeParams,params);
  uint8_t* function = nullptr;
  if(compileMethodBuilder(&builder,(void**)(&function))==0) {
    return reinterpret_cast<JITedFunction>(function);
  }else{
    return nullptr;
  }
}

OMR::JitBuilder::IlType* functionReturnType(interp::DefinedFunc* fn,interp::Environment& env,
			       wabt::jit::AOTTypeDictionary &types)
{
    const auto& result_types = env.GetFuncSignature(fn->sig_index)->result_types;

    if(result_types.empty()) {
      return types.toIlType<void>();
    } else {
      return TypeFieldType(result_types.front());
    }
}

OMR::JitBuilder::IlType* TypeFieldType(Type t) {
  AOTTypeDictionary types_;
  switch (t) {
    case Type::I32:
      return types_.toIlType<int32_t>();
    case Type::I64:
      return types_.toIlType<int64_t>();
    case Type::F32:
      return types_.toIlType<float>();
    case Type::F64:
      return types_.toIlType<double>();
    default:
      //TR_ASSERT_FATAL(false, "Invalid primitive type");
      return nullptr;
  }
}

OMR::JitBuilder::IlType* TypeFieldType(const char* t) {
  AOTTypeDictionary types_;
  if(strcmp(t, "i32") == 0) {
      return types_.toIlType<int32_t>();
  } else if(strcmp(t, "i64") == 0) {
      return types_.toIlType<int64_t>();
  } else if(strcmp(t, "f32") == 0) {
      return types_.toIlType<float>();
  } else if(strcmp(t, "f64") == 0) {
      return types_.toIlType<double>();
  }

  //TR_ASSERT_FATAL(false, "Invalid primitive type");
  return nullptr;
}
*/
}
}
