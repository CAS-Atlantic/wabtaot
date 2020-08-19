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

#ifndef FUNCTIONBUILDER_HPP
#define FUNCTIONBUILDER_HPP

#include "aot-type-dictionary.h"
#include "ilgen/BytecodeBuilder.hpp"
#include "ilgen/MethodBuilder.hpp"
#include "ilgen/VirtualMachineOperandStack.hpp"

#include "src/interp/interp.h"

#include <map>
#include <vector>
#include <type_traits>


extern void getCompiledFunction(const char *,void (**)());
extern wabt::interp::Environment *getEnvironment();

namespace wabt {
namespace aot {

using namespace wabt::interp;
  
class AOTManager;
class FunctionImport;

class AOTFunctionBuilder : public TR::MethodBuilder {
 public:
  AOTFunctionBuilder(interp::Thread*, interp::DefinedFunc*, std::string&&,
		     AOTTypeDictionary*, Environment&, AOTManager&);
  
  bool buildIL() override;

  TR::IlValue* popReturnValue(TR::IlBuilder*);
  void pushReturnValue(interp::Func*, TR::IlBuilder*, TR::IlValue*);
  void pushReturnValue(Index, TR::IlBuilder*, TR::IlValue*);
  void pushParams();
  
  virtual ~AOTFunctionBuilder() {}
  
  /**
   * @brief Generate push to the VM operand stack
   * @param b is the builder object used to generate the code
   * @param type is the name of the field in the Value union corresponding to the type of the value being pushed
   * @param value is the IlValue representing the value being pushed
   */
  void Push(TR::IlBuilder* b, const char* type, TR::IlValue* value); //, const uint8_t* pc);

  /**
   * @brief Generate pop from the interpreter stack
   * @param b is the builder object used to generate the code
   * @param type is the name of the field in the Value union corresponding to the type of the value being popped
   * @return an IlValue representing the popped value
   */
  TR::IlValue* Pop(TR::IlBuilder* b, const char* type);

  /**
   * @brief Drop a number of values from the interpreter stack, optionally keeping the top value of the stack
   * @param b is the builder object used to generate the code
   * @param drop_count is the number of values to drop from the stack
   * @param keep_count is 1 to keep the top value intact and 0 otherwise
   */
  void DropKeep(TR::IlBuilder* b, uint32_t drop_count, uint8_t keep_count);

  /**
   * @brief Generate load of pointer to a vlue on the interpreter stack by an index
   * @param b is the builder object used to generate the code
   * @param depth is the index from the top of the stack
   * @return and IlValue representing a pointer to the value on the stack
   *
   * JitBuilder does not currently represent unions as value types. This a problem
   * for this function because it cannot simply return an IlValue representing
   * the union. As workaround, it will generate a load of the *base address* of
   * the union, instead of loading the union directly. This behaviour differs
   * from `Thread::Pick()` and users must take this into account.
   */
  TR::IlValue* Pick(Index depth);
  uint32_t pickLocalOffset();
  
  void defineFunction(const std::string&, interp::DefinedFunc*);
  void defineImportFunction(const std::string& name, FunctionImport &import);

  interp::DefinedFunc* getFn() {
    return fn_;
  }
  
  const std::string& getName() const {
    return fn_name_;
  }

  static uint64_t CallIndirectHelper(Index table_index, Index sig_index, Index entry_index);
  static uint32_t GrowMemory(uint32_t,uint32_t);

 private:
  struct BytecodeWorkItem {
    TR::BytecodeBuilder* builder;
    const uint8_t* pc;
    TR::VirtualMachineOperandStack* stack_;
    uint32_t stackCount_;

    BytecodeWorkItem(TR::BytecodeBuilder* builder, const uint8_t* pc, 
		     TR::VirtualMachineOperandStack* stack,  uint32_t stackCount)
    : builder(builder), pc(pc), stack_(stack), stackCount_(stackCount)
    {}
  };

  TR::IlType* functionReturnType(interp::Func*);
  
  template <typename T>
  const char* TypeFieldName() const;

  const char* TypeFieldName(Type) const;
  const char* TypeFieldName(TR::DataType) const;
  TR::IlType* TypeFieldType(Type, TR::IlBuilder*) const;
  TR::IlType* TypeFieldType(const char*, TR::IlBuilder*) const;

  TR::IlValue* Const(TR::IlBuilder* b, const interp::TypedValue* v) const;

  template <typename T, typename TResult = T, typename TOpHandler>
  void EmitBinaryOp(TR::IlBuilder* b, /* const uint8_t* pc,*/ TOpHandler h);

  template <typename T, typename TResult = T, typename TOpHandler>
  void EmitUnaryOp(TR::IlBuilder* b, /* const uint8_t* pc,*/ TOpHandler h);

  template <typename T>
  void EmitIntDivide(TR::IlBuilder* b);

  template <typename T>
  void EmitUnsignedIntDivide(TR::IlBuilder* b);

  template <typename T>
  void EmitIntRemainder(TR::IlBuilder* b);

  template <typename T>
  TR::IlValue* EmitMemoryPreAccess(TR::IlBuilder* b);

  void returnWithError(TR::IlBuilder*);
  
  void EmitTrap(TR::IlBuilder* b, interp::Result);
  // void EmitCheckTrap(TR::IlBuilder* b, TR::IlValue* result);
  void EmitTrapIf(TR::IlBuilder* b, TR::IlValue* condition, interp::Result);

  template <typename F>
  TR::IlValue* EmitIsNan(TR::IlBuilder* b, TR::IlValue* value);

  template <typename ToType, typename FromType>
  void EmitTruncation(TR::IlBuilder* b);
  template <typename ToType, typename FromType>
  void EmitUnsignedTruncation(TR::IlBuilder* b);

  TR::IlValue* calculateGlobalIndex(TR::IlBuilder* b, const uint8_t**);
  TR::IlValue* calculateMemoryIndex(TR::IlBuilder* b, const uint8_t**);
  
  template <typename>
  TR::IlValue* CalculateShiftAmount(TR::IlBuilder* b, TR::IlValue* amount);

  using Result_t = std::underlying_type<wabt::interp::Result>::type;

//  static Result_t CallHelper(wabt::interp::Thread* th, wabt::interp::IstreamOffset offset, uint8_t* current_pc);

  


  //static Result_t CallHostHelper(wabt::interp::Thread* th, Index func_index);

  //  static void* MemoryTranslationHelper(interp::Thread* th, uint32_t memory_id, uint64_t address, uint32_t size);

  std::vector<BytecodeWorkItem> workItems_;

  AOTTypeDictionary* types_;
  
  interp::Thread* thread_;
  interp::DefinedFunc* fn_;
  
  std::string fn_name_;
  
  Environment& env_;
  static Environment* envPointer;
  AOTManager& aotManager_;
  
  TR::IlType* const valueType_;
  TR::IlType* const pValueType_;
  TR::IlType* const ppValueType_;

  TR::VirtualMachineOperandStack* stack_;
  uint32_t stackCount_ = 0;
  uint32_t localsCount_ = 0;
  
  TR::IlType* returnType_;
  
  std::vector<std::string> param_names_;
  std::vector<TR::IlType*> param_types_;

  std::vector<std::string> global_names_;
  std::vector<std::string> mem_names_;

  struct PreviousCompilerState {
    TR::BytecodeBuilder* b;
    TR::VirtualMachineOperandStack* stack;
    const uint8_t* pc;
    uint32_t stack_count;

    PreviousCompilerState(TR::BytecodeBuilder* b, TR::VirtualMachineOperandStack* stack,
			  const uint8_t* pc, uint32_t stack_count)
     : b(b), stack(stack), pc(pc), stack_count(stack_count) {}
  };

  std::vector<PreviousCompilerState> stackOfStacks_;
  
  bool Emit(TR::BytecodeBuilder* b, const uint8_t* istream, const uint8_t* pc);
  int64_t regist_;
};

class FunctionImport {
  public:
    FunctionImport(interp::Func *fn):fn_(fn){}

    std::vector<TR::IlType*> param_types_;
    interp::Func *fn_;
};

class AOTManager {
 public:
  void push_back_FB(uint32_t offset, std::unique_ptr<AOTFunctionBuilder>&& b,
		    std::unique_ptr<AOTTypeDictionary>&& t)
  {
    func_index_[offset] = {std::move(b), std::move(t)};
  }

  void push_back_import(std::string name, interp::Func* fn){
    import_index_.emplace_back(name,fn);
  }
  
  AOTFunctionBuilder& getFB(uint32_t i) {
    return *func_index_[i].first;
  }

  AOTTypeDictionary* getTD(uint32_t i) {
    return func_index_[i].second.get();
  }
  
  void broadcastNames() {
    for(auto& builder_kv: func_index_) {
      for(auto& inner_kv: func_index_) {
	auto& builder_fn = builder_kv.second.first;
	inner_kv.second.first->defineFunction(builder_fn->getName(),
					      builder_fn->getFn());
      }
    }
  }

  void broadcastImports() {
    for(auto& builder: func_index_) {
      for(auto& import: import_index_) {
        builder.second.first->defineImportFunction(import.first,import.second);
      }
    }
  }

 private:
  std::map<uint32_t, std::pair<std::unique_ptr<AOTFunctionBuilder>,
                               std::unique_ptr<AOTTypeDictionary>>>
    func_index_;

  std::vector<std::pair<std::string,FunctionImport>> import_index_;
};



}
}

#endif
