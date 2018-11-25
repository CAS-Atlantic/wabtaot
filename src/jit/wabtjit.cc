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

#include <cstddef>

#include "wabtjit.h"
#include "type-dictionary.h"
#include "function-builder.h"

extern int32_t internal_compileMethodBuilder(TR::MethodBuilder * methodBuilder, void ** entryPoint);

namespace wabt {
namespace jit {

JITedFunction compile(interp::Thread* thread, interp::DefinedFunc* fn) {
  TypeDictionary types;
  FunctionBuilder builder(thread, fn, &types);
  void* function = nullptr;

  if (internal_compileMethodBuilder(&builder, &function) == 0) {
    return reinterpret_cast<JITedFunction>(function);
  } else {
    return nullptr;
  }
}

}
}
