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

#ifndef WABTJIT_HPP
#define WABTJIT_HPP

#include "src/common.h"
//#include "type-dictionary.h"
#include "src/interp/interp.h"

namespace wabt {
namespace jit {

using JITedFunction = interp::Result (*)();

//wabt::Result compileAOT(interp::Thread*, interp::Environment&);
JITedFunction compile(interp::Thread* thread, interp::Func*);
/*OMR::JitBuilder::IlType* TypeFieldType(const char* t);
OMR::JitBuilder::IlType* TypeFieldType(Type t);
OMR::JitBuilder::IlType* functionReturnType(interp::DefinedFunc* fn,interp::Environment& env,
			       AOTTypeDictionary &types);
JITedFunction loadCompiled(interp::Thread* thread, interp::Func* fn,
			   interp::Environment& env);
JITedFunction loadThunk(interp::Thread* thread, interp::Func* fn,
			   interp::Environment& env);
*/
}
}

#endif // WABTJIT_HPP
