#include "aot-function-builder.h"
#include "aot-type-dictionary.h"
#include "trap-with.h"
#include "src/cast.h"
#include "src/interp/interp.h"
#include "src/interp/interp-internal.h"
//#include "/home/petar/wasmjit-omr/third_party/omr/compiler/ilgen/VirtualMachineOperandStack.hpp"
//#include "infra/Assert.hpp"
#include "ilgen/VirtualMachineState.hpp"
#include "ilgen/VirtualMachineRegister.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <string.h>

namespace wabt {

namespace aot {

#define CHECK_TRAP_IN_HELPER(...)                \
  do {                                           \
    wabt::interp::Result result = (__VA_ARGS__); \
    if (result != wabt::interp::Result::Ok) {    \
      return static_cast<Result_t>(result);      \
    }                                            \
  } while (0)
#define TRAP(type) return static_cast<Result_t>(wabt::interp::Result::Trap##type)
#define TRAP_UNLESS(cond, type) TRAP_IF(!(cond), type)
#define TRAP_IF(cond, type)  \
  do {                       \
    if (WABT_UNLIKELY(cond)) \
      TRAP(type);            \
  } while (0)

Environment* AOTFunctionBuilder::envPointer = 0;

AOTFunctionBuilder::AOTFunctionBuilder(interp::Thread* thread, interp::DefinedFunc* fn,
                                       std::string&& fn_name, AOTTypeDictionary* types,
                                       Environment& env, AOTManager& aotManager)
  : TR::MethodBuilder(types),
    types_(types),
    thread_(thread),
    fn_(fn),
    fn_name_(std::move(fn_name)),
    env_(env),
    aotManager_(aotManager),
    valueType_(Int64),
    pValueType_(types_->PointerTo(Int64)),
    ppValueType_(types_->PointerTo(pValueType_))
{

  DefineLine(__LINE__);
  DefineFile(__FILE__);
  DefineName(fn->dbg_name_.c_str());

  DefineReturnType(wabt::jit::toIlType<Result_t>(types));

  DefineFunction("sqrtf", __FILE__, "0",
                 reinterpret_cast<void*>(static_cast<float (*)(float)>(sqrtf)),
                 Float,
                 1,
                 Float);
  DefineFunction("copysignf", __FILE__, "0",
                 reinterpret_cast<void*>(static_cast<float (*)(float, float)>(copysignf)),
                 Float,
                 2,
                 Float,
                 Float);
  DefineFunction("sqrt", __FILE__, "0",
                 reinterpret_cast<void*>(static_cast<double (*)(double)>(sqrt)),
                 Double,
                 1,
                 Double);
  DefineFunction("copysign", __FILE__, "0",
                 reinterpret_cast<void*>(static_cast<double (*)(double, double)>(copysign)),
                 Double,
                 2,
                 Double,
                 Double);
  DefineFunction("trapWith", __FILE__, "0",
		 reinterpret_cast<void*>(static_cast<void (*)(Result_t)>(trapWith)),
		 NoType,
		 1,
		 Int32);

  DefineFunction("CallIndi", __FILE__, "0",
		 reinterpret_cast<void*>(CallIndirectHelper),
		 Int64,
		 3,
		 Int64, Int64, Int64);
  DefineFunction("Popcount", __FILE__, "0",
		 reinterpret_cast<void*>(static_cast<int(*)(unsigned)>(wabt::Popcount)),
		 Int32,
		 1,
		 Int32);
  DefineFunction("Popcountll", __FILE__, "0",
		 reinterpret_cast<void*>(static_cast<int(*)(unsigned long long)>(wabt::Popcount)),
		 Int64,
		 1,
		 Int32);
  DefineFunction("GrowMem", __FILE__, "0",
		 reinterpret_cast<void*>(GrowMemory),
		 Int32,
		 1,
		 Int32,
		 Int32);
  DefineFunction("funpr", __FILE__, "0",
		 reinterpret_cast<void*>(sqrt),
		 NoType,
		 1,
		 Int64);
  
  envPointer = &env_;
  returnType_ = functionReturnType(fn_);

  auto memories_size = env_.GetMemoryCount();
  auto globals_size = env_.GetGlobalCount();
  auto param_count = env_.GetFuncSignature(fn_->sig_index)->param_types.size();
  int total_size = param_count;

  if (memories_size > 0)
  		total_size++;
  if (globals_size > 0)
  		total_size++;
  param_names_.reserve(total_size);

      int arg = 0;

  for(const auto& t: env_.GetFuncSignature(fn_->sig_index)->param_types) {
    char param[6]; // ie, "p6" is the sixth parameter.
    sprintf(param, "p%d", arg++);

    param_names_.push_back(param);
    TR::IlType* tt = TypeFieldType(t,this);

    DefineParameter(param_names_.back().data(), tt);
    param_types_.push_back(tt);
  }
  
  arg = 0;
  global_names_.reserve(globals_size);
  for(const auto& g: env_.globals_) {
    char global_name[6];
    sprintf(global_name,"g%d",arg++);
    TR::IlType *gt = TypeFieldType(g.typed_value.type,this);

    global_names_.push_back(global_name);
    DefineGlobal(global_names_.back().data(),gt,reinterpret_cast<void*>(&(env_.globals_.data()[arg-1].typed_value.value)));
  }
  arg = 0;
  mem_names_.reserve(memories_size);
  for(const auto& m: env_.memories_) {
    char mem_name[6];
    sprintf(mem_name,"m%d",arg++);
//    TR::IlType *mt = TypeFieldType(ppValue);

    mem_names_.push_back(mem_name);
    DefineGlobal(mem_names_.back().data(),types_->PointerTo(Int8),reinterpret_cast<void*>(env_.mems+arg-1));
    //it must be a pointer to the value, as in globals
  }

  if(env_.GetTableCount()) {
    DefineGlobal("Params",types_->PointerTo(Int64),reinterpret_cast<void*>(env_.indirectCallParams));
  }
  DefineLocal("SelectionVar",Int32);

  DefineReturnType(returnType_);

}

void AOTFunctionBuilder::pushParams() {
  
  auto memories_size = env_.GetMemoryCount();
  auto globals_size = env_.GetGlobalCount();
  int arg = 0;
  
  for(const auto& t: env_.GetFuncSignature(fn_->sig_index)->param_types) {
    Push(this, TypeFieldName(t), Load(param_names_[arg++].data()));
  }
}

/*
 Build a struct containing the return types of the function, as its fields.
 */

TR::IlType* AOTFunctionBuilder::functionReturnType(interp::Func* fn)
{
    const auto& result_types = env_.GetFuncSignature(fn->sig_index)->result_types;

    if(result_types.empty()) {
      return NoType;
    } else {
      return TypeFieldType(result_types.front(), this);
    }
}

void AOTFunctionBuilder::defineFunction(const std::string& name, interp::DefinedFunc* fn)
{
  if(fn == fn_) return;

  TR::IlType* result_type = fn == fn_ ? returnType_ : functionReturnType(fn);
  auto& builder_fn = aotManager_.getFB(fn->offset);

  DefineFunction(name.c_str(), __FILE__, "0",
		 reinterpret_cast<void*>(18), // this is a magic number that makes trampoline lookup work.
		 result_type,
		 builder_fn.param_types_.size(),
		 static_cast<TR::IlType**>(builder_fn.param_types_.data()));
}

void AOTFunctionBuilder::defineImportFunction(const std::string& name, FunctionImport &import)
{

  TR::IlType* result_type = functionReturnType(import.fn_);
  if(import.param_types_.size() != env_.GetFuncSignature(import.fn_->sig_index)->param_types.size()){
    for(const auto& t: env_.GetFuncSignature(import.fn_->sig_index)->param_types) {
    
      TR::IlType* tt = TypeFieldType(t,this);

      import.param_types_.push_back(tt);
    }
  }
  DefineFunction(name.c_str(), __FILE__, "0",
		 reinterpret_cast<void*>(18), // this is a magic number that makes trampoline lookup work.
		 result_type,
		 import.param_types_.size(),
		 static_cast<TR::IlType**>(import.param_types_.data()));
}


uint64_t AOTFunctionBuilder::CallIndirectHelper(Index table_index, Index sig_index, Index entry_index) {
  using namespace wabt::interp;
  
  Environment *env = ::getEnvironment();

//  Index table_index = reinterpret_cast<Index>(params[0]);
  Table* table = &env->tables_[table_index];
//  Index sig_index = reinterpret_cast<Index>(params[1]);
//  Index entry_index = reinterpret_cast<Index>(params[2]);
  TRAP_IF(entry_index >= table->func_indexes.size(), UndefinedTableIndex);
  Index func_index = table->func_indexes[entry_index];
  TRAP_IF(func_index == kInvalidIndex, UninitializedTableElement);
  Func* func = env->funcs_[func_index].get();
  TRAP_UNLESS(env->FuncSignaturesAreEqual(func->sig_index, sig_index),
              IndirectCallSignatureMismatch);
//  assert(env->GetFuncSignature(sig_index)->param_types.size() == count-3);
//  count-=3;
  auto count = env->GetFuncSignature(sig_index)->param_types.size();  
 // printf("FI:%d\n",func_index);
 // for (auto aa:env->GetFuncSignature(sig_index)->param_types) {
  //  printf("%d\n",aa);
  //}

  if (func->is_host) {
    //auto result = static_cast<Result_t>(th->CallHost(cast<HostFunc>(func)));
    //if (result != static_cast<Result_t>(interp::Result::Ok))
      //return result;
  } else {
    uint64_t (*fn)();
    ::getCompiledFunction(func->dbg_name_.c_str(),reinterpret_cast<void(**)()>(&fn));

    //since calls are made regularly, every possible number and type of parameters requires a case
    switch(count) {
      case 0:
        return fn();
      case 1: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t)>(fn);
        return funct(param1);
      }
      case 2: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto param2 = env->indirectCallParams[1];
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t,uint64_t)>(fn);
        return funct(param2,param1);
      }
      case 3: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto param2 = env->indirectCallParams[1];
        auto param3 = env->indirectCallParams[2];
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t,uint64_t,uint64_t)>(fn);
        return funct(param3,param2,param1);
      }
      case 4: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto param2 = env->indirectCallParams[1];
        auto param3 = env->indirectCallParams[2];
	auto param4 = env->indirectCallParams[3];
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t,uint64_t,uint64_t,uint64_t)>(fn);
        return funct(param4,param3,param2,param1);
      }
      case 5: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto param2 = env->indirectCallParams[1];
        auto param3 = env->indirectCallParams[2];
	auto param4 = env->indirectCallParams[3];
	auto param5 = env->indirectCallParams[4];
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t,uint64_t,uint64_t,uint64_t,uint64_t)>(fn);
        return funct(param5,param4,param3,param2,param1);
      }
      case 6: {
        auto param1 = env->indirectCallParams[0]; //might be problems with order of variables
        auto param2 = env->indirectCallParams[1];
        auto param3 = env->indirectCallParams[2];
        auto param4 = env->indirectCallParams[3]; //might be problems with order of variables
        uint64_t param5 = env->indirectCallParams[4];
        auto param6 = env->indirectCallParams[5];
	double dParam5 = 0.0;
	memcpy(&dParam5,&param5,8);
        auto funct = reinterpret_cast<uint64_t(*)(uint64_t,double,uint64_t,uint64_t,uint64_t,uint64_t)>(fn);
        return funct(param6,dParam5,param4,param3,param2,param1);
      }

      default:
          throw std::runtime_error("Too many arguments!");
    }
    return fn();
  }
  return static_cast<Result_t>(interp::Result::Ok);
}

//Currently the memory is not actually resized, only the data on the number of pages
uint32_t AOTFunctionBuilder::GrowMemory(uint32_t mem, uint32_t grow_pages) {
  //printf("Grow by: %ud",grow_pages);
  Memory *memory = envPointer->GetMemory(mem);
  uint32_t old_page_size = memory->page_limits.initial;
  uint32_t new_page_size = old_page_size + grow_pages;
  //memory->data.resize(new_page_size * WABT_PAGE_SIZE);
  memory->page_limits.initial = new_page_size;
  return old_page_size;
}


bool AOTFunctionBuilder::buildIL() {
  setVMState(new TR::VirtualMachineState());

  // expects a non-NULL Compilation object to exist, so must be
  // constructed here, at compile time
  auto regAddr = ConstAddress(&regist_);
  TR::VirtualMachineRegister *xx = new TR::VirtualMachineRegister(this,"reg",ppValueType_,8,regAddr);
  stack_ = new TR::VirtualMachineOperandStack(this, 64, valueType_, xx);

  pushParams();

  const uint8_t* istream = thread_->GetIstream();

  workItems_.emplace_back(OrphanBytecodeBuilder(0,
						const_cast<char*>(interp::ReadOpcodeAt(&istream[fn_->offset]).GetName())),
                          &istream[fn_->offset],stack_,stackCount_);
  AppendBuilder(workItems_[0].builder);

  int32_t next_index;

  for(;;) {
    if ((next_index = GetNextBytecodeFromWorklist()) != -1) {
      auto& work_item = workItems_[next_index];
      stack_ = work_item.stack_;
      stackCount_ = work_item.stackCount_;

      if (!Emit(work_item.builder, istream, work_item.pc))
	return false;
    } else if(!stackOfStacks_.empty()) {
      auto prev_state = stackOfStacks_.back();
      stackOfStacks_.pop_back();

      int32_t next_index = static_cast<int32_t>(workItems_.size());

      workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
						    const_cast<char*>(interp::ReadOpcodeAt(prev_state.pc).GetName())),
			      prev_state.pc,prev_state.stack, prev_state.stack_count);

      prev_state.b->AddFallThroughBuilder(workItems_[next_index].builder);
      //stack_ = prev_state.stack;
      //stackCount_ = prev_state.stack_count;
    } else {
      break;
    }
  }

  return true;
}

void AOTFunctionBuilder::Push(TR::IlBuilder* b, const char* type, TR::IlValue* value)
{
  //TODO: should probably compare to valueType_ here, if that's
  //possible. I'm not sure if a simple pointer comparison will
  //work. IlTypes* for primitives might not be singleton values.
/*
  TR::IlValue* value_wrapper;
  if(value->getDataType().isDouble())
    value_wrapper = b->ConvertBitsTo(valueType_, value);
  else if( value->getDataType().isFloatingPoint()) {
    auto convert1 = b->ConvertBitsTo(Int32, value);
    value_wrapper = b->ConvertTo(Int64, convert1);
    //auto convert1 = b->ConvertTo(Double, value);
    //value_wrapper = b->ConvertBitsTo(Int64, convert1);
    //value_wrapper = b->ConvertTo(valueType_, value);
  }
  else
    value_wrapper = strcmp(type, "i64") ? b->ConvertTo(valueType_, value) : value;
*/
  //Bitcast seems to provide the best conversions for this compiler, currently
  
  TR::IlValue* value_wrapper = strcmp(type, "i64") ? b->BitcastTo(valueType_, value) : value;
  stackCount_++;
  stack_->Push(b, value_wrapper);
}


TR::IlValue* AOTFunctionBuilder::Pop(TR::IlBuilder* b, const char* type) {
  auto* value = stack_->Pop(b);
  stackCount_--;
/*
  if(TypeFieldType(type,b) == Double)
    return b->ConvertBitsTo(TypeFieldType(type,b), value);
  else if( TypeFieldType(type,b) == Float ){
    auto convert1 = b->ConvertBitsTo(Double, value);
    return b->ConvertTo(Float, convert1);
    //auto convert1 = b->ConvertTo(Int32, value);
    //return b->ConvertBitsTo(Float, convert1);
    return b->ConvertTo(TypeFieldType(type,b), value);
  }
  else
    return strcmp("i64", type) ? b->ConvertTo(TypeFieldType(type,b), value) : value;
*/
  return strcmp("i64", type) ? b->BitcastTo(TypeFieldType(type,b), value) : value;
}

void AOTFunctionBuilder::DropKeep(TR::IlBuilder* b, uint32_t drop_count, uint8_t keep_count) {
  //TR_ASSERT(keep_count <= 1, "Invalid keep count");
  //TR_ASSERT(stackCount_ >= drop_count + keep_count, "Invalid drop count");

  if(keep_count == 1) {
      auto* top = stack_->Pop(b);
      stack_->Drop(b, drop_count-1);
      stack_->Push(b, top);
  } else {
    stack_->Drop(b, drop_count);
  }

  stackCount_ -= drop_count;
}

/**
 * @brief Generate load from the interpreter stack by an index
 *
 * The generate code should be equivalent to:
 *
 * return &value_stack_[value_stack_top_ - depth];
 */
TR::IlValue* AOTFunctionBuilder::Pick(Index depth) {
  return stack_->Pick(depth-1);
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<int32_t>() const {
  return "i32";
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<uint32_t>() const {
  return "i32";
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<int64_t>() const {
  return "i64";
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<uint64_t>() const {
  return "i64";
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<float>() const {
  return "f32";
}

template <>
const char* AOTFunctionBuilder::TypeFieldName<double>() const {
  return "f64";
}

const char* AOTFunctionBuilder::TypeFieldName(Type t) const {
  switch (t) {
    case Type::I32:
      return TypeFieldName<int32_t>();
    case Type::I64:
      return TypeFieldName<int64_t>();
    case Type::F32:
      return TypeFieldName<float>();
    case Type::F64:
      return TypeFieldName<double>();
    default:
      TR_ASSERT_FATAL(false, "Invalid primitive type");
      return nullptr;
  }
}

const char* AOTFunctionBuilder::TypeFieldName(TR::DataType dt) const {
  switch (dt.getDataType()) {
    case TR::Int32:
      return TypeFieldName<int32_t>();
    case TR::Int64:
      return TypeFieldName<int64_t>();
    case TR::Float:
      return TypeFieldName<float>();
    case TR::Double:
      return TypeFieldName<double>();
    default:
      TR_ASSERT_FATAL(false, "Invalid primitive type %s", dt.toString());
      return nullptr;
  }
}

TR::IlType* AOTFunctionBuilder::TypeFieldType(Type t, TR::IlBuilder *b) const {
  switch (t) {
    case Type::I32:
      return wabt::jit::toIlType<int32_t>(b->typeDictionary());
    case Type::I64:
      return wabt::jit::toIlType<int64_t>(b->typeDictionary());
    case Type::F32:
      return wabt::jit::toIlType<float>(b->typeDictionary());
    case Type::F64:
      return wabt::jit::toIlType<double>(b->typeDictionary());
    default:
      //TR_ASSERT_FATAL(false, "Invalid primitive type");
      return nullptr;
  }
}

TR::IlType* AOTFunctionBuilder::TypeFieldType(const char* t, TR::IlBuilder *b) const {
  if(strcmp(t, "i32") == 0) {
      return wabt::jit::toIlType<int32_t>(b->typeDictionary());
  } else if(strcmp(t, "i64") == 0) {
      return wabt::jit::toIlType<int64_t>(b->typeDictionary());
  } else if(strcmp(t, "f32") == 0) {
      return wabt::jit::toIlType<float>(b->typeDictionary());
  } else if(strcmp(t, "f64") == 0) {
      return wabt::jit::toIlType<double>(b->typeDictionary());
  }

  //TR_ASSERT_FATAL(false, "Invalid primitive type");
  return nullptr;
}

TR::IlValue* AOTFunctionBuilder::Const(TR::IlBuilder* b, const interp::TypedValue* v) const {
  switch (v->type) {
    case Type::I32:
      return b->ConstInt32(v->value.i32);
    case Type::I64:
      return b->ConstInt64(v->value.i64);
    case Type::F32:
      return b->ConstFloat(Bitcast<float>(v->value.f32_bits));
    case Type::F64:
      return b->ConstDouble(Bitcast<double>(v->value.f64_bits));
    default:
      TR_ASSERT_FATAL(false, "Invalid primitive type");
      return nullptr;
  }
}

template <typename T, typename TResult, typename TOpHandler>
void AOTFunctionBuilder::EmitBinaryOp(TR::IlBuilder* b, TOpHandler h) {
  auto* rhs = Pop(b, TypeFieldName<T>());
  auto* lhs = Pop(b, TypeFieldName<T>());

  Push(b, TypeFieldName<TResult>(), h(lhs, rhs)); //, pc);
}

template <typename T, typename TResult, typename TOpHandler>
void AOTFunctionBuilder::EmitUnaryOp(TR::IlBuilder* b, TOpHandler h) {
  Push(b, TypeFieldName<TResult>(), h(Pop(b, TypeFieldName<T>())));//, pc);
}

template <typename T>
void AOTFunctionBuilder::EmitIntDivide(TR::IlBuilder* b) {
  static_assert(std::is_integral<T>::value,
                "EmitIntDivide only works on integral types");

  EmitBinaryOp<T>(b, [&](TR::IlValue* dividend, TR::IlValue* divisor) {
    EmitTrapIf(b,
    b->        EqualTo(divisor, b->Const(static_cast<T>(0))),
	       interp::Result::TrapIntegerDivideByZero);

    EmitTrapIf(b,
    b->        And(
    b->            EqualTo(dividend, b->Const(std::numeric_limits<T>::min())),
    b->            EqualTo(divisor, b->Const(static_cast<T>(-1)))),
	       interp::Result::TrapIntegerOverflow);

    return b->Div(dividend, divisor);
  });
}

template <typename T>
void AOTFunctionBuilder::EmitUnsignedIntDivide(TR::IlBuilder* b) {
  static_assert(std::is_integral<T>::value,
                "EmitIntDivide only works on integral types");

  EmitBinaryOp<T>(b, [&](TR::IlValue* dividend, TR::IlValue* divisor) {
    EmitTrapIf(b,
    b->        EqualTo(divisor, b->Const(static_cast<T>(0))),
	       interp::Result::TrapIntegerDivideByZero);

    EmitTrapIf(b,
    b->        And(
    b->            EqualTo(dividend, b->Const(std::numeric_limits<T>::min())),
    b->            EqualTo(divisor, b->Const(static_cast<T>(-1)))),
	       interp::Result::TrapIntegerOverflow);

    return b->UnsignedDiv(dividend, divisor);
  });
}

template <typename T>
void AOTFunctionBuilder::EmitIntRemainder(TR::IlBuilder* b) {//, const uint8_t* pc) {
  static_assert(std::is_integral<T>::value,
                "EmitIntRemainder only works on integral types");

  EmitBinaryOp<T>(b, [&](TR::IlValue* dividend, TR::IlValue* divisor) {
    EmitTrapIf(b,
    b->        EqualTo(divisor, b->Const(static_cast<T>(0))),
	       interp::Result::TrapIntegerDivideByZero);

    TR::IlValue* return_value = b->Const(static_cast<T>(0));

    TR::IlBuilder* div_no_ovf_path = nullptr;
    b->IfThen(&div_no_ovf_path,
    b->       Or(
    b->           NotEqualTo(dividend, b->Const(std::numeric_limits<T>::min())),
    b->           NotEqualTo(divisor, b->Const(static_cast<T>(-1)))));
    div_no_ovf_path->StoreOver(return_value,
                               div_no_ovf_path->Rem(dividend, divisor));

    return return_value;
  });
}

TR::IlValue* AOTFunctionBuilder::calculateMemoryIndex(TR::IlBuilder* b, const uint8_t** pc)
{
  TR::IlType *pMemoryType = types_->PointerTo(Int8);
  TR::IlType *ppMemoryType = types_->PointerTo(pMemoryType);
  auto mem_id = b->ConstInt64(static_cast<uint64_t>(ReadU32(pc)));
  auto memory = b->IndexAt(ppMemoryType, b->Load("memories"), mem_id);
  auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(pc)));

  auto address = b->Add(Pop(b, "i64"), offset);
  auto location = b->IndexAt(pMemoryType, b->LoadAt(ppMemoryType, memory), address);
  EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
	     interp::Result::TrapMemoryAccessOutOfBounds);

  return location;
}

TR::IlValue* AOTFunctionBuilder::calculateGlobalIndex(TR::IlBuilder* b, const uint8_t** pc)
{
  auto glob_id = b->ConstInt64(static_cast<uint64_t>(ReadU32(pc)));
  auto global = b->IndexAt(ppValueType_, b->Load("globals"), glob_id);

  return b->IndexAt(pValueType_, b->LoadAt(ppValueType_, global), b->ConstInt64(0));
}

void AOTFunctionBuilder::returnWithError(TR::IlBuilder* b) {
  const auto& return_types = env_.GetFuncSignature(fn_->sig_index)->result_types;

  if(return_types.empty()) {
    b->Return();
  } else {
    const auto& return_type = return_types.front();

    switch (return_type) {
    case Type::I32:
      return b->Return(b->ConstInt32(-1));
    case Type::I64:
      return b->Return(b->ConstInt64(-1));
    case Type::F32:
      return b->Return(b->ConstFloat(-1.0f));
    case Type::F64:
      return b->Return(b->ConstDouble(-1.0));
    default:
      throw std::runtime_error("invalid WASM return type!");
    }
  }
}

void AOTFunctionBuilder::EmitTrap(TR::IlBuilder* b, interp::Result r) {
  b->Call("trapWith", 1, b->Const(static_cast<int32_t>(r)));
  returnWithError(b);
}

void AOTFunctionBuilder::EmitTrapIf(TR::IlBuilder* b, TR::IlValue* condition, interp::Result result) {
  TR::IlBuilder* trap_handler = nullptr;

  b->IfThen(&trap_handler, condition);
  EmitTrap(trap_handler, result);
}

template <>
TR::IlValue* AOTFunctionBuilder::EmitIsNan<float>(TR::IlBuilder* b, TR::IlValue* value) {
  return b->GreaterThan(
         b->           And(
         b->               ConvertBitsTo(Int32, value),
         b->               ConstInt32(0x7fffffffU)),
         b->           ConstInt32(0x7f800000U));
}

template <>
TR::IlValue* AOTFunctionBuilder::EmitIsNan<double>(TR::IlBuilder* b, TR::IlValue* value) {
  return b->GreaterThan(
         b->           And(
         b->               ConvertBitsTo(Int64, value),
         b->               ConstInt64(0x7fffffffffffffffULL)),
         b->           ConstInt64(0x7ff0000000000000ULL));
}

template <typename ToType, typename FromType>
void AOTFunctionBuilder::EmitTruncation(TR::IlBuilder* b) {//, const uint8_t* pc) {
  static_assert(std::is_floating_point<FromType>::value,
		"FromType in EmitTruncation call must be a floating point type");

  auto* value = Pop(b, TypeFieldName<FromType>());

  // TRAP_IF is NaN
  EmitTrapIf(b,
             EmitIsNan<FromType>(b, value),
	     interp::Result::TrapInvalidConversionToInteger);

  // TRAP_UNLESS conversion is in range
  EmitTrapIf(b,
  b->        Or(
  b->           LessThan(value,
  b->                    Const(static_cast<FromType>(std::numeric_limits<ToType>::lowest()))),
  b->           GreaterThan(value,
  b->                       Const(static_cast<FromType>(std::numeric_limits<ToType>::max())))),
	     interp::Result::TrapIntegerOverflow);

  auto* target_type = wabt::jit::toIlType<ToType>( b->typeDictionary());

  // this could be optimized using templates or constant expressions,
  // but the compiler should be able to simplify this anyways
  auto* new_value = std::is_unsigned<ToType>::value ? b->ConvertBitsTo(target_type, value)
    : b->ConvertTo(target_type, value);
//  auto new_value = b->BitcastTo(target_type,value);

  Push(b, TypeFieldName<ToType>(), new_value);
}

/**
 * @brief Special case of EmitTruncation for unsigned integers as target type
 *
 * This function is designed to handle the case of truncating to an unsigned integer type.
 * When the target type is an unsigned integer type smaller than 64-bits, the floating-point
 * value can be safely truncated to a *signed* 64-bit integer and then converted to
 * the target type.
 */
template <typename ToType, typename FromType>
void AOTFunctionBuilder::EmitUnsignedTruncation(TR::IlBuilder* b) { // , const uint8_t* pc) {
  static_assert(std::is_floating_point<FromType>::value, "FromType in EmitTruncation call must be a floating point type");
  static_assert(std::is_integral<ToType>::value, "ToType in EmitUnsignedTruncation call must be an integer type");
  static_assert(std::is_unsigned<ToType>::value, "ToType in EmitUnsignedTruncation call must be unsigned");

  auto* value = Pop(b, TypeFieldName<FromType>());

  // TRAP_IF is NaN
  EmitTrapIf(b,
             EmitIsNan<FromType>(b, value),
	     interp::Result::TrapInvalidConversionToInteger);

  // TRAP_UNLESS conversion is in range
  EmitTrapIf(b,
  b->        Or(
  b->           LessThan(value,
  b->                    Const(static_cast<FromType>(std::numeric_limits<ToType>::lowest()))),
  b->           GreaterThan(value,
  b->                       Const(static_cast<FromType>(std::numeric_limits<ToType>::max())))),
	     interp::Result::TrapIntegerOverflow);

  auto* target_type = wabt::jit::toIlType<ToType>( b->typeDictionary());
  auto* new_value = b->UnsignedConvertTo(target_type, b->ConvertTo(Int64, value));

  Push(b, TypeFieldName<ToType>(), new_value);
}

// return a struct of type (fn_name_ + "_return_type").
TR::IlValue* AOTFunctionBuilder::popReturnValue(TR::IlBuilder* b) {
  const auto& result_types = env_.GetFuncSignature(fn_->sig_index)->result_types;

  if(result_types.empty())
    return nullptr;
  else {
    return Pop(b, TypeFieldName(result_types.front()));
  }
}

void AOTFunctionBuilder::pushReturnValue(Func* builder, TR::IlBuilder* b,
					 TR::IlValue* returnValue)
{
  const auto& result_types = env_.GetFuncSignature(builder->sig_index)->result_types;

  if(result_types.empty())
    return;
  else {
    Push(b, TypeFieldName(result_types.front()), returnValue);
  }
}

void AOTFunctionBuilder::pushReturnValue(Index sig_index, TR::IlBuilder* b,
					 TR::IlValue* returnValue)
{
  const auto& result_types = env_.GetFuncSignature(sig_index)->result_types;

  if(result_types.empty())
    return;
  else {
    Push(b, TypeFieldName(result_types.front()), returnValue);
  }
}

/*
template <typename ToType, typename FromType>
void FunctionBuilder::EmitSaturatingTruncation(TR::IlBuilder* b, VirtualStack* stack) {
  static_assert(std::is_floating_point<FromType>::value, "FromType in EmitTruncation call must be a floating point type");
  static_assert(std::is_integral<ToType>::value, "ToType in EmitTruncation call must be an integer type");
  static_assert(std::is_signed<ToType>::value, "ToType in EmitTruncation call must be signed");

  auto* value = stack->Pop();
  auto* result = b->Const(static_cast<ToType>(0));

  TR::IlBuilder* non_nan_path = nullptr;

  b->IfThen(&non_nan_path, b->EqualTo(EmitIsNan<FromType>(b, value), b->ConstInt32(0)));

  TR::IlBuilder* too_high_path = nullptr;
  TR::IlBuilder* not_too_high_path = nullptr;

  non_nan_path->IfThenElse(&too_high_path, &not_too_high_path,
  non_nan_path->           GreaterThan(value,
  non_nan_path->                       Const(static_cast<FromType>(std::numeric_limits<ToType>::max()))));
  too_high_path->StoreOver(result, too_high_path->Const(std::numeric_limits<ToType>::max()));

  TR::IlBuilder* too_low_path = nullptr;
  TR::IlBuilder* not_too_low_path = nullptr;

  not_too_high_path->IfThenElse(&too_low_path, &not_too_low_path,
  not_too_high_path->           LessThan(value,
  not_too_high_path->                    Const(static_cast<FromType>(std::numeric_limits<ToType>::lowest()))));
  too_low_path->StoreOver(result, too_low_path->Const(std::numeric_limits<ToType>::lowest()));

  auto* target_type = toIlType<ToType>(b->typeDictionary());

  // this could be optimized using templates or constant expressions,
  // but the compiler should be able to simplify this anyways
  auto* new_value = std::is_unsigned<ToType>::value ? not_too_low_path->UnsignedConvertTo(target_type, value)
                                                    : not_too_low_path->ConvertTo(target_type, value);
  not_too_low_path->StoreOver(result, new_value);

  stack->Push(result);
}

template <typename ToType, typename FromType>
void FunctionBuilder::EmitUnsignedSaturatingTruncation(TR::IlBuilder* b, VirtualStack* stack) {
  static_assert(std::is_floating_point<FromType>::value, "FromType in EmitTruncation call must be a floating point type");
  static_assert(std::is_integral<ToType>::value, "ToType in EmitUnsignedTruncation call must be an integer type");
  static_assert(std::is_unsigned<ToType>::value, "ToType in EmitUnsignedTruncation call must be unsigned");

  using ToTypeSigned = typename std::make_signed<ToType>::type;

  auto* value = stack->Pop();
  auto* result = b->Const(static_cast<ToTypeSigned>(0));

  TR::IlBuilder* non_nan_path = nullptr;

  b->IfThen(&non_nan_path, b->EqualTo(EmitIsNan<FromType>(b, value), b->ConstInt32(0)));

  TR::IlBuilder* too_high_path = nullptr;
  TR::IlBuilder* not_too_high_path = nullptr;

  non_nan_path->IfThenElse(&too_high_path, &not_too_high_path,
  non_nan_path->           GreaterThan(value,
  non_nan_path->                       Const(static_cast<FromType>(std::numeric_limits<ToType>::max()))));
  too_high_path->StoreOver(result, too_high_path->Const(static_cast<ToTypeSigned>(std::numeric_limits<ToType>::max())));

  TR::IlBuilder* too_low_path = nullptr;
  TR::IlBuilder* not_too_low_path = nullptr;

  not_too_high_path->IfThenElse(&too_low_path, &not_too_low_path,
  not_too_high_path->           LessThan(value,
  not_too_high_path->                    Const(static_cast<FromType>(std::numeric_limits<ToType>::lowest()))));
  too_low_path->StoreOver(result, too_low_path->Const(static_cast<ToTypeSigned>(std::numeric_limits<ToType>::lowest())));

  auto* target_type = toIlType<ToType>(b->typeDictionary());

  auto* new_value = not_too_low_path->UnsignedConvertTo(target_type, not_too_low_path->ConvertTo(Int64, value));
  not_too_low_path->StoreOver(result, new_value);

  stack->Push(result);
}*/

template <typename T>
TR::IlValue* AOTFunctionBuilder::CalculateShiftAmount(TR::IlBuilder* b, TR::IlValue* amount) {
  return b->UnsignedConvertTo(Int32,
         b->                  And(amount, b->Const(static_cast<T>(sizeof(T) * 8 - 1))));
}

bool AOTFunctionBuilder::Emit(TR::BytecodeBuilder* b,
                           const uint8_t* istream,
                           const uint8_t* pc) {
  Opcode opcode = interp::ReadOpcode(&pc);
  TR_ASSERT(!opcode.IsInvalid(), "Invalid opcode");

  switch (opcode) {
    case Opcode::Select: {
      auto* sel = Pop(b, "i64");
      auto* false_value = Pop(b, "i64");
      auto* true_value = Pop(b, "i64");

      TR::IlBuilder* true_path = nullptr;

      b->IfThen(&true_path, sel);
      true_path->StoreOver(false_value, true_value);

      Push(b,"i64",false_value);
      break;
    }

    case Opcode::Br: {
      auto target = &istream[ReadU32(&pc)];
      auto it = std::find_if(workItems_.cbegin(), workItems_.cend(),
			     [&](const BytecodeWorkItem& b) {
			       return target == b.pc;
			     });
      if (it != workItems_.cend() && *((uint32_t*)(it->pc))!=Opcode::LocalTee && *((uint32_t*)(it->pc))!=Opcode::LocalSet) {
        b->AddFallThroughBuilder(it->builder);
      } else {
        int32_t next_index = static_cast<int32_t>(workItems_.size());
        workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
                                                      const_cast<char*>(ReadOpcodeAt(target).GetName())),
                                target,stack_,stackCount_);
        b->AddFallThroughBuilder(workItems_[next_index].builder);
      }
      return true;
    }

    case Opcode::BrTable: {
        Index num_targets = ReadU32(&pc);
        IstreamOffset table_offset = ReadU32(&pc);
        b->Store("SelectionVar",Pop(b,"i32"));
	JBCase **cases = new JBCase*[num_targets];
	for(uint32_t i=0;i<num_targets;i++) {
	  const uint8_t* entry = istream + table_offset + (i*WABT_TABLE_ENTRY_SIZE);
          IstreamOffset new_pc;
          uint32_t drop_count;
          uint32_t keep_count;

          ReadTableEntryAt(entry, &new_pc, &drop_count, &keep_count);
	  int32_t next_index = static_cast<int32_t>(workItems_.size());
          workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
                                                      const_cast<char*>(ReadOpcodeAt(reinterpret_cast<uint8_t*>(&new_pc)).GetName())),
                                  &istream[new_pc],new TR::VirtualMachineOperandStack(stack_),stackCount_);
	  auto nextBuilder = static_cast<TR::IlBuilder*>(workItems_[workItems_.size()-1].builder);
	  cases[i] = b->MakeCase(i,&nextBuilder,0);
	  //auto nextBuilder1 = workItems_[workItems_.size()-1].builder;
	  b->AddSuccessorBuilder(&workItems_[workItems_.size()-1].builder);
	}
	const uint8_t* entry = istream + table_offset + (num_targets*WABT_TABLE_ENTRY_SIZE);
          IstreamOffset new_pc;
          uint32_t drop_count;
          uint32_t keep_count;
          ReadTableEntryAt(entry, &new_pc, &drop_count, &keep_count);
	  int32_t next_index = static_cast<int32_t>(workItems_.size());
	  
          workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
                                                      const_cast<char*>(ReadOpcodeAt(reinterpret_cast<uint8_t*>(&new_pc)).GetName())),
                                  &istream[new_pc],new TR::VirtualMachineOperandStack(stack_),stackCount_);
	  auto nextBuilder = static_cast<TR::IlBuilder*>(workItems_[workItems_.size()-1].builder);
	   b->AddSuccessorBuilder(&workItems_[workItems_.size()-1].builder);
	  b->TableSwitch(b->Load("SelectionVar"),&nextBuilder,false,num_targets,cases);
	  delete cases;
	  return true;
    }

    // case Opcode::BrIf: This opcode is never generated as it's always
    // transformed into a BrUnless. So, there's no need to handle it.

    case Opcode::Return: {
      auto* value = popReturnValue(b); // of type fn_name + "_return_type"
      //auto* value = Pop(b, TypeFieldName(result_type.front()));

      if(value == nullptr) {
	b->Return();
      } else {
        b->Return(value);
      }

      return true;
    }

    case Opcode::Unreachable:
      //EmitTrap(b, interp::Result::TrapUnreachable);
      //return true;
      break;

    case Opcode::I32Const: {
      auto* val = b->ConstInt32(ReadU32(&pc));
      Push(b, "i32", val);
      break;
    }

    case Opcode::I64Const: {
      auto* val = b->ConstInt64(ReadU64(&pc));
      Push(b, "i64", val);
      break;
    }

    case Opcode::F32Const: {
      auto* val = b->ConstFloat(ReadUx<float>(&pc));
      Push(b, "f32", val);
      break;
    }

    case Opcode::F64Const: {
      auto* val = b->ConstDouble(ReadUx<double>(&pc));
      Push(b, "f64", val);
      break;
    }

    case Opcode::GlobalGet: {
      //throw std::runtime_error("AOTFunctionBuilder: get_global not supported");
      /*
      interp::Global* g = thread_->env()->GetGlobal(ReadU32(&pc));

      // The type of value stored in a global will never change, so we're safe
      // to use the current type of the global.
      const char* type_field = TypeFieldName(g->typed_value.type);

      if (g->mutable_) {
        // TODO(thomasbc): Can the address of a Global change at runtime?
        auto* addr = b->Const(&g->typed_value.value);
        Push(b, type_field, b->LoadIndirect("Value", type_field, addr));//, pc);
      } else {
        // With immutable globals, we can just substitute their actual value as
        // a constant at compile-time.
        Push(b, type_field, Const(b, &g->typed_value));//, pc);
      }
      */
//      auto* addr = calculateGlobalIndex(b, &pc); // comes out as i64.//need this as it moves the wasm bytecode cursor(goes to next instruction)
//      Push(b, "i64", b->LoadAt(pValueType_, addr));
      auto index = ReadU32(&pc);
      Push(b,TypeFieldName(env_.globals_[index].typed_value.type),b->Load(global_names_[index].data()));

      break;
    }

    case Opcode::GlobalSet: {
      //auto* address = calculateGlobalIndex(b, &pc); 
      //TODO FIX ONLY TYPE, SHOULD BE MORE TYPES
      auto index = ReadU32(&pc);
      auto* value = Pop(b, TypeFieldName(env_.globals_[index].typed_value.type));
//  	  b->StoreAt(address, value);
      b->Store(global_names_[index].data(),value);
      /*
      interp::Global* g = thread_->env()->GetGlobal(ReadU32(&pc));
      assert(g->mutable_);

      // See note for get_global
      const char* type_field = TypeFieldName(g->typed_value.type);

      // TODO(thomasbc): Can the address of a Global change at runtime?
      auto* addr = b->Const(&g->typed_value.value);

      b->StoreIndirect("Value", type_field, addr, Pop(b, type_field));
      */
      break;
    }

    case Opcode::LocalGet: {
      // note: to work around JitBuilder's lack of support unions as value types,
      // just copy a field that's the size of the entire union
      auto* local_addr = Pick(ReadU32(&pc));
      Push(b, "i64", local_addr); //b->LoadIndirect("Value", "i64", local_addr));
      
      break;
    }

    case Opcode::LocalSet: {
      // see note for GetLocal
      auto* value = Pop(b, "i64");
      auto* local_addr = Pick(ReadU32(&pc));
      b->StoreOver(local_addr, value);
      // b->StoreIndirect("Value", "i64", local_addr, value);
      break;
    }

    case Opcode::LocalTee: {
      auto* local_addr = Pick(ReadU32(&pc));
      b->StoreOver(local_addr, Pick(1));
      break;
    }

    case Opcode::InterpCallHost:
    case Opcode::Call: {
      auto offset = ReadU32(&pc);
      auto meta_it = env_.jit_meta_.find(offset);

    if(meta_it != env_.jit_meta_.end()) {
	auto* fn = meta_it->second.wasm_fn;
    //auto *fn = env_.GetFunc(offset);
	auto& builder = aotManager_.getFB(fn->offset);

	//std::vector<TR::IlValue*> args;
	int size = env_.GetFuncSignature(fn->sig_index)->param_types.size();
	//TR::IlValue **args1 = new TR::IlValue*[size]();

	//uint64_t namee = 0;
	//memcpy(&namee, fn->dbg_name_.c_str(), 3);
	//auto ilname = b->ConstInt64(namee);
	//args1[0] = ilname;

	//b->Call("funpr",1,args1);
	TR::IlValue **args = new TR::IlValue*[size]();

	//for(const auto& t: env_.GetFuncSignature(fn->sig_index)->param_types) {
	int i= size;
	for(auto t = env_.GetFuncSignature(fn->sig_index)->param_types.rbegin();
	    t!=env_.GetFuncSignature(fn->sig_index)->param_types.rend(); t++) {
	  args[--i] = Pop(b, TypeFieldName(*t));
	}

 	auto* value = b->Call(fn->dbg_name_.c_str(), size, args);
	pushReturnValue(fn, b, value);
	delete args;
//	aotManager_.addCallToRegistry(fn_name_,builder.fn_name_);
      } else {
	throw std::runtime_error("Call: function not found!");
      }
      
      break;
    }

  case Opcode::CallIndirect: {
    
//    auto th_addr = b->ConstAddress(thread_);
    auto table_index = b->ConstInt64(ReadU32(&pc));
    auto sig = ReadU32(&pc);
    auto sig_index = b->ConstInt64(sig);
    auto entry_index = Pop(b, "i64");
//    auto current_pc = b->Const(pc);
      
    int size = env_.GetFuncSignature(sig)->param_types.size();
	TR::IlValue **args = new TR::IlValue*[3]();
    
    int i = 0;
    
    auto *array = b->Load("Params");
    for(auto t = env_.GetFuncSignature(sig)->param_types.begin();
	    t!=env_.GetFuncSignature(sig)->param_types.end(); t++) {
	  auto location = b->IndexAt(types_->PointerTo(Int64), array, b->ConstInt64(i++));
      //b->StoreAt(location, Pop(b, TypeFieldName(*t)));
	  b->StoreAt(location, Pop(b, "i64"));
	}
    args[2] = entry_index;
    args[1] = sig_index;
    args[0] = table_index;

    auto *value = b->Call("CallIndi", 3, args);
    pushReturnValue(sig, b, value);
      
    break;
    }

    // case Opcode::InterpCallHost: {
    //   throw std::runtime_error("interpreted host calls not supported");
    //   //    Index func_index = ReadU32(&pc);

    //   // TODO: again, more of the same.

    //   /*
    //   b->Store("result",
    //   b->      Call("CallHostHelper", 2,
    //   b->           ConstAddress(thread_),
    //   b->           ConstInt32(func_index)));

    //   EmitCheckTrap(b, b->Load("result"), nullptr);
    //   */
    //   break;
    // }

    case Opcode::I32Load8S: 

    case Opcode::I32Load8U: {
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int8), location);
      Push(b, "i32", b->ConvertTo(Int32, value));
      break;
    }

    case Opcode::I32Load16S: 

    case Opcode::I32Load16U: {
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int16), location);
      Push(b, "i32", b->ConvertTo(Int32, value));
      break;
    }

    case Opcode::I32Load:
    case Opcode::F32Load: {
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int32), location);
      Push(b, "i32", value);
      break;
    }

    case Opcode::I64Load8S: 

    case Opcode::I64Load8U: {
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int8), location);
      Push(b, "i64", b->ConvertTo(Int64, value));
      break;
    }

    case Opcode::I64Load16S: 

    case Opcode::I64Load16U: {
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int16), location);
      Push(b, "i64", b->ConvertTo(Int64, value));
      break;
    }

    case Opcode::I64Load32S: 

    case Opcode::I64Load32U:{
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      // auto memsize = b->ConstInt64(64*1024);
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto dynamicAddr = Pop(b, "i64");
      //EmitTrapIf(b,b->UnsignedGreaterThan(dynamicAddr,
	//	 b->Sub(b->Sub(memsize,offset),b->ConstInt64(4))),
	  //   interp::Result::TrapMemoryAccessOutOfBounds);//pValueType is always 64-bit...
      auto address = b->Add(dynamicAddr, offset);
      
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      TR::IlValue *value = b->LoadAt(types_->PointerTo(Int32), location);
      Push(b, "i64", b->ConvertTo(Int64, value));
      break;
    }

    case Opcode::I64Load:
    case Opcode::F64Load: {
      //auto* addr = calculateMemoryIndex(b, &pc); // comes out as i64.
      //Push(b, "i64", b->LoadAt(pValueType_, addr));
      auto index = ReadU32(&pc);
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
	     interp::Result::TrapMemoryAccessOutOfBounds);
      Push(b, "i64", b->LoadAt(pValueType_, location));
      break;
    }

    case Opcode::I32Store8: 
    case Opcode::I64Store8: {
      auto index = ReadU32(&pc);
      auto *value = Pop(b,"i64"); 
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      //auto *value = Pop(b,"i64");
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
           interp::Result::TrapMemoryAccessOutOfBounds);
      b->StoreAt(location, b->ConvertTo(Int8, value));
      break;
    }

    case Opcode::I32Store16:  
    case Opcode::I64Store16: {
      auto index = ReadU32(&pc);
      auto *value = Pop(b,"i64"); 
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      //auto *value = Pop(b,"i64");
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
           interp::Result::TrapMemoryAccessOutOfBounds);
      b->StoreAt(location, b->ConvertTo(Int16, value));
      break;
    }

    case Opcode::I64Store32: {
      auto index = ReadU32(&pc);
      auto *value = Pop(b,"i64"); 
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      //auto *value = Pop(b,"i64");
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
           interp::Result::TrapMemoryAccessOutOfBounds);
      b->StoreAt(location, b->ConvertTo(Int32, value));
      break;
    }

    case Opcode::I32Store:
    case Opcode::F32Store:{
      auto index = ReadU32(&pc);
      auto *value = Pop(b,"i64"); 
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      //auto *value = Pop(b,"i64");
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
           interp::Result::TrapMemoryAccessOutOfBounds);
      b->StoreAt(location, b->ConvertTo(Int32, value));
      break;
    }

    case Opcode::I64Store:
    case Opcode::F64Store: {
//      auto* value = Pop(b, "i64");
//      b->StoreAt(calculateMemoryIndex(b, &pc), value);
      auto index = ReadU32(&pc);
      auto *value = Pop(b,"i64"); 
      auto *mem = b->Load(mem_names_[index].data());
      auto offset = b->ConstInt64(static_cast<uint64_t>(ReadU32(&pc)));
      auto address = b->Add(Pop(b, "i64"), offset);
      //auto *value = Pop(b,"i64");
      auto location = b->IndexAt(types_->PointerTo(Int8), mem, address);
      EmitTrapIf(b,b->EqualTo(location,b->ConstAddress(nullptr)),
           interp::Result::TrapMemoryAccessOutOfBounds);
      b->StoreAt(location, value);
      break;
    }

    case Opcode::I32Add:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Add(lhs, rhs);
      });
      break;

    case Opcode::I32Sub:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Sub(lhs, rhs);
      });
      break;

    case Opcode::I32Mul:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Mul(lhs, rhs);
      });
      break;

    case Opcode::I32DivU:
      EmitUnsignedIntDivide<int32_t>(b);
      break;

    case Opcode::I32DivS:
      EmitIntDivide<int32_t>(b);
      break;

    case Opcode::I32RemU:
    case Opcode::I32RemS:
      EmitIntRemainder<int32_t>(b);
      break;

    case Opcode::I32And:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->And(lhs, rhs);
      });
      break;

    case Opcode::I32Or:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Or(lhs, rhs);
      });
      break;

    case Opcode::I32Xor:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Xor(lhs, rhs);
      });
      break;

    case Opcode::I32Shl:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->ShiftL(lhs, CalculateShiftAmount<int32_t>(b, rhs));
      });
      break;

    case Opcode::I32ShrS:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->ShiftR(lhs, CalculateShiftAmount<int32_t>(b, rhs));
      });
      break;

    case Opcode::I32ShrU:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedShiftR(lhs, CalculateShiftAmount<int32_t>(b, rhs));
      });
      break;

    case Opcode::I32Rotl:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        auto* amount = CalculateShiftAmount<int32_t>(b, rhs);

        return b->Or(
        b->          ShiftL(lhs, amount),
        b->          UnsignedShiftR(lhs, b->Sub(b->ConstInt32(32), amount)));
      });
      break;

    case Opcode::I32Rotr:
      EmitBinaryOp<int32_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        auto* amount = CalculateShiftAmount<int32_t>(b, rhs);

        return b->Or(
        b->          UnsignedShiftR(lhs, amount),
        b->          ShiftL(lhs, b->Sub(b->ConstInt32(32), amount)));
      });
      break;

    case Opcode::I32Eqz:
      EmitUnaryOp<int32_t, int>(b, [&](TR::IlValue* val) {
        return b->EqualTo(val, b->ConstInt32(0));
      });
      break;

    case Opcode::I32Eq:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->EqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32Ne:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->NotEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32LtS:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessThan(lhs, rhs);
      });
      break;

    case Opcode::I32LtU:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedLessThan(lhs, rhs);
      });
      break;

    case Opcode::I32GtS:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterThan(lhs, rhs);
      });
      break;

    case Opcode::I32GtU:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedGreaterThan(lhs, rhs);
      });
      break;

    case Opcode::I32LeS:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32LeU:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedLessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32GeS:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32GeU:
      EmitBinaryOp<int32_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedGreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64Add:
        EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
          return b->Add(lhs, rhs);
        });
        break;

    case Opcode::I64Sub:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Sub(lhs, rhs);
      });
      break;

    case Opcode::I64Mul:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Mul(lhs, rhs);
      });
      break;
     
    case Opcode::I64DivU:
      EmitUnsignedIntDivide<int64_t>(b);
      break;

      
    case Opcode::I64DivS: // RETURN
      EmitIntDivide<int64_t>(b); //, pc);
      break;

    case Opcode::I64RemU:
    case Opcode::I64RemS:
      EmitIntRemainder<int64_t>(b);//, pc);
      break;

    case Opcode::I64And:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->And(lhs, rhs);
      });
      break;

    case Opcode::I64Or:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Or(lhs, rhs);
      });
      break;

    case Opcode::I64Xor:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Xor(lhs, rhs);
      });
      break;

    case Opcode::I64Shl:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->ShiftL(lhs, CalculateShiftAmount<int64_t>(b, rhs));
      });
      break;

    case Opcode::I64ShrS:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->ShiftR(lhs, CalculateShiftAmount<int64_t>(b, rhs));
      });
      break;

    case Opcode::I64ShrU:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedShiftR(lhs, CalculateShiftAmount<int64_t>(b, rhs));
      });
      break;

    case Opcode::I64Rotl:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        auto* amount = CalculateShiftAmount<int64_t>(b, rhs);

        return b->Or(
        b->          ShiftL(lhs, amount),
        b->          UnsignedShiftR(lhs, b->Sub(b->ConstInt32(64), amount)));
      });
      break;

    case Opcode::I64Rotr:
      EmitBinaryOp<int64_t>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        auto* amount = CalculateShiftAmount<int64_t>(b, rhs);

        return b->Or(
        b->          UnsignedShiftR(lhs, amount),
        b->          ShiftL(lhs, b->Sub(b->ConstInt32(64), amount)));
      });
      break;

    case Opcode::I64Eqz:
      EmitUnaryOp<int64_t, int>(b, [&](TR::IlValue* val) {
        return b->EqualTo(val, b->ConstInt64(0));
      });
      break;

    case Opcode::I64Eq:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->EqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64Ne:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->NotEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64LtS:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessThan(lhs, rhs);
      });
      break;

    case Opcode::I64LtU:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedLessThan(lhs, rhs);
      });
      break;

    case Opcode::I64GtS:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterThan(lhs, rhs);
      });
      break;

    case Opcode::I64GtU:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedGreaterThan(lhs, rhs);
      });
      break;

    case Opcode::I64LeS:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64LeU:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedLessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64GeS:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I64GeU:
      EmitBinaryOp<int64_t, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->UnsignedGreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F32Abs:
      EmitUnaryOp<float>(b, [&](TR::IlValue* value) {
        auto* return_value = b->Copy(value);

        TR::IlBuilder* zero_path = nullptr;
        TR::IlBuilder* nonzero_path = nullptr;
        TR::IlBuilder* neg_path = nullptr;

        // We have to check explicitly for 0.0, since abs(-0.0) is 0.0.
        b->IfThenElse(&zero_path, &nonzero_path, b->EqualTo(value, b->ConstFloat(0)));
        zero_path->StoreOver(return_value, zero_path->ConstFloat(0));

        nonzero_path->IfThen(&neg_path,
	nonzero_path->       LessThan(value, nonzero_path->ConstFloat(0)));
        neg_path->           StoreOver(return_value,
	neg_path->                     Mul(value,
        neg_path->                         ConstFloat(-1)));

        return return_value;
      });
      break;

    case Opcode::F32Neg:
      EmitUnaryOp<float>(b, [&](TR::IlValue* value) {
        return b->Mul(value, b->ConstFloat(-1));
      });
      break;

    case Opcode::F32Sqrt:
      EmitUnaryOp<float>(b, [&](TR::IlValue* value) {
        return b->Call("sqrtf", 1, value);
      });
      break;

    case Opcode::F32Add:
      EmitBinaryOp<float>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Add(lhs, rhs);
      });
      break;

    case Opcode::F32Sub:
      EmitBinaryOp<float>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Sub(lhs, rhs);
      });
      break;

    case Opcode::F32Mul:
      EmitBinaryOp<float>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Mul(lhs, rhs);
      });
      break;

    case Opcode::F32Div:
      EmitBinaryOp<float>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Div(lhs, rhs);
      });
      break;

    case Opcode::F32Copysign:
      EmitBinaryOp<float>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Call("copysignf", 2, lhs, rhs);
      });
      break;

    case Opcode::F32Eq:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->EqualTo(lhs, rhs);
      });
      break;

    case Opcode::F32Ne:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->NotEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F32Lt:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessThan(lhs, rhs);
      });
      break;

    case Opcode::F32Le:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F32Gt:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterThan(lhs, rhs);
      });
      break;

    case Opcode::F32Ge:
      EmitBinaryOp<float, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F64Abs:
      EmitUnaryOp<double>(b, [&](TR::IlValue* value) {
        auto* return_value = b->Copy(value);

        TR::IlBuilder* zero_path = nullptr;
        TR::IlBuilder* nonzero_path = nullptr;
        TR::IlBuilder* neg_path = nullptr;

        // We have to check explicitly for 0.0, since abs(-0.0) is 0.0.
        b->IfThenElse(&zero_path, &nonzero_path,
	b->           EqualTo(value, b->ConstDouble(0)));
        zero_path->   StoreOver(return_value, zero_path->ConstDouble(0));

        nonzero_path->IfThen(&neg_path,
        nonzero_path->       LessThan(value,
        nonzero_path->       ConstDouble(0)));

        neg_path->StoreOver(return_value,
        neg_path->          Mul(value,
	neg_path->          ConstDouble(-1)));

        return return_value;
      });
      break;

    case Opcode::F64Neg:
      EmitUnaryOp<double>(b, [&](TR::IlValue* value) {
        return b->Mul(value, b->ConstDouble(-1));
      });
      break;

    case Opcode::F64Sqrt:
      EmitUnaryOp<double>(b, [&](TR::IlValue* value) {
        return b->Call("sqrt", 1, value);
      });
      break;

    case Opcode::F64Add:
      EmitBinaryOp<double>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Add(lhs, rhs);
      });
      break;

    case Opcode::F64Sub:
      EmitBinaryOp<double>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Sub(lhs, rhs);
      });
      break;

    case Opcode::F64Mul:
      EmitBinaryOp<double>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Mul(lhs, rhs);
      });
      break;

    case Opcode::F64Div:
      EmitBinaryOp<double>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Div(lhs, rhs);
      });
      break;

    case Opcode::F64Copysign:
      EmitBinaryOp<double>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->Call("copysign", 2, lhs, rhs);
      });
      break;

    case Opcode::F64Eq:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->EqualTo(lhs, rhs);
      });
      break;

    case Opcode::F64Ne:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->NotEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F64Lt:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessThan(lhs, rhs);
      });
      break;

    case Opcode::F64Le:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->LessOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::F64Gt:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterThan(lhs, rhs);
      });
      break;

    case Opcode::F64Ge:
      EmitBinaryOp<double, int>(b, [&](TR::IlValue* lhs, TR::IlValue* rhs) {
        return b->GreaterOrEqualTo(lhs, rhs);
      });
      break;

    case Opcode::I32WrapI64: {
      auto* value = Pop(b, "i64");
      Push(b, "i32",
      b->  ConvertTo(Int32, value));
	   //           pc);
      break;
    }

    case Opcode::I64ExtendI32S: {
      auto* value = Pop(b, "i32");
      Push(b, "i64",
      b->  ConvertTo(Int64, value));
      break;
    }

    case Opcode::I64ExtendI32U: {
      auto* value = Pop(b, "i32");
      Push(b, "i64",
      b->  UnsignedConvertTo(Int64, value));
	   //     pc);
      break;
    }

    case Opcode::F32DemoteF64: {
      auto* value = Pop(b, "f64");
      Push(b, "f32",
      b->  ConvertTo(Float, value));
	   //     pc);
      break;
    }

    case Opcode::F64PromoteF32: {
      auto* value = Pop(b, "f32");
      Push(b, "f64",
      b->  ConvertTo(Double, value));
	   //           pc);
      break;
    }

    case Opcode::I32Extend8S: {
      auto* value = b->ConvertTo(Int32, b->ConvertTo(Int8, Pop(b, "i32")));
      Push(b, "i32", value);//, pc);
      break;
    }

    case Opcode::I32Extend16S: {
      auto* value = b->ConvertTo(Int32, b->ConvertTo(Int16, Pop(b, "i32")));
      Push(b, "i32", value);//, pc);
      break;
    }

    case Opcode::I64Extend8S: {
      auto* value = b->ConvertTo(Int32, b->ConvertTo(Int8, Pop(b, "i32")));
      Push(b, "i32", value);//, pc);
      break;
    }

    case Opcode::I64Extend16S: {
      auto* value = b->ConvertTo(Int32, b->ConvertTo(Int16, Pop(b, "i32")));
      Push(b, "i32", value);//, pc);
      break;
    }

    case Opcode::I64Extend32S: {
      auto* value = b->ConvertTo(Int64, b->ConvertTo(Int32, Pop(b, "i64")));
      Push(b, "i64", value);//, pc);
      break;
    }

    case Opcode::F32ConvertI32S: {
      auto* value = b->ConvertBitsTo(Float, Pop(b, "i32"));
      Push(b, "f32", value);//, pc);
      break;
    }

    case Opcode::F32ConvertI32U: {
      auto* value = b->ConvertBitsTo(Float,b->UnsignedConvertTo(Int32, Pop(b, "i32")));
      Push(b, "f32", value);//, pc);
      break;
    }

    case Opcode::F32ConvertI64S: {
      auto* value = b->ConvertTo(Float, Pop(b, "i64"));
      Push(b, "f32", value);//, pc);
      break;
    }

    case Opcode::F32ConvertI64U: {
      auto* value = b->ConvertTo(Float,b->UnsignedConvertTo(Int64, Pop(b, "i64")));
      Push(b, "f32", value);//, pc);
      break;
    }

    case Opcode::F64ConvertI32S: {
      auto* value = b->ConvertTo(Double, Pop(b, "i32"));
      Push(b, "f64", value);//, pc);
      break;
    }

    case Opcode::F64ConvertI32U: {
      auto* value = b->ConvertTo(Double,b->UnsignedConvertTo(Int32, Pop(b, "i32")));
      Push(b, "f64", value);//, pc);
      break;
    }

    case Opcode::F64ConvertI64S: {
      auto* value = b->ConvertBitsTo(Double, Pop(b, "i64"));
      Push(b, "f64", value);//, pc);
      break;
    }

    case Opcode::F64ConvertI64U: {
      auto* value = b->ConvertBitsTo(Double,b->UnsignedConvertTo(Int64, Pop(b, "i64")));
      Push(b, "f64", value);//, pc);
      break;
    }

    case Opcode::F32ReinterpretI32: {
      //auto* value = b->ConvertTo(Float, Pop(b, "i32"));
      auto* value = b->ConvertBitsTo(Float, Pop(b, "i32"));
      Push(b, "f32", value);//, pc);
      break;
    }

    case Opcode::I32ReinterpretF32: {
      //auto* value = b->ConvertTo(Int32, Pop(b, "f32"));
      auto* value = b->ConvertBitsTo(Int32, Pop(b, "f32"));
      Push(b, "i32", value);//, pc);
      break;
    }

    case Opcode::F64ReinterpretI64: {
      //auto* value = b->ConvertTo(Double, Pop(b, "i64"));
      auto* value = b->ConvertBitsTo(Double, Pop(b, "i64"));
      Push(b, "f64", value);//, pc);
      break;
    }

    case Opcode::I64ReinterpretF64: {
      //auto* value = b->ConvertTo(Int64, Pop(b, "f64"));
      auto* value = b->ConvertBitsTo(Int64, Pop(b, "f64"));
      Push(b, "i64", value);//, pc);
      break;
    }

    case Opcode::I32TruncF32S:
      EmitTruncation<int32_t, float>(b);//pc);
      break;

    case Opcode::I32TruncF32U:
      EmitUnsignedTruncation<uint32_t, float>(b);//pc);
      break;

    case Opcode::I32TruncF64S:
      EmitTruncation<int32_t, double>(b);//pc);
      break;

    case Opcode::I32TruncF64U:
      EmitUnsignedTruncation<uint32_t, double>(b);//pc);
      break;

    case Opcode::I64TruncF32S:
      EmitTruncation<int64_t, float>(b);//pc);
      break;

//    UNSIGNED TYPE NOT HANDLED
//    case Opcode::I64TruncUF32:
//      EmitTruncation<uint64_t, float>(b, pc);
//      break;

    case Opcode::I64TruncF64S:
      EmitTruncation<int64_t, double>(b);//pc);
      break;

//    UNSIGNED TYPE NOT HANDLED
//    case Opcode::I64TruncUF64:
//      EmitTruncation<uint64_t, double>(b, pc);
//      break;

    case Opcode::MemorySize:
        Push(b, "i64", b->ConstInt64(64*1024));
        break;

    case Opcode::MemoryGrow: {
      TR::IlValue **args = new TR::IlValue*[2]();
      args[0] = ConstInt32(ReadU32(&pc));
      args[1] = Pop(b,"i32");
      auto* value = b->Call("GrowMem",2,args);
      Push(b,"i64",value);
      break;
    }

    case Opcode::InterpAlloca: {
      auto count = ReadU32(&pc);

      for(Index i = 0; i < count; ++i) {
          Push(b, "i64", b->ConstInt64(0));
          localsCount_++;
      }

      break;
    }

   case Opcode::I64Popcnt:{
     
      TR::IlValue **args = new TR::IlValue*[1]{Pop(b,"i64")};
      auto* value = b->Call("Popcountll", 1, args);
      Push(b,"i64",value);
      delete args;
      break;
   }
    
    case Opcode::I32Popcnt: {

      TR::IlValue **args = new TR::IlValue*[1]{Pop(b,"i32")};
      auto* value = b->Call("Popcount", 1, args);
      Push(b,"i32",value);
      delete args;
      break;
    }

    case Opcode::InterpBrUnless: {
      auto target = &istream[ReadU32(&pc)];
      auto condition = Pop(b, "i32");
      auto it = std::find_if(workItems_.begin(), workItems_.end(),
			     [&](const BytecodeWorkItem& b) {
			       return target == b.pc;
			     });

      if (it != workItems_.end() && *((uint32_t*)(it->pc))!=Opcode::LocalTee && *((uint32_t*)(it->pc))!=Opcode::LocalSet) {
        b->IfCmpEqualZero(&it->builder, condition);
      } else {
        int32_t next_index = static_cast<int32_t>(workItems_.size());
        workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
                                                      const_cast<char*>(ReadOpcodeAt(target).GetName())),
                                target,stack_,stackCount_);
        b->IfCmpEqualZero(&workItems_[next_index].builder, condition);
      }

      TR::VirtualMachineOperandStack *prev_stack = new TR::VirtualMachineOperandStack(stack_);
      stackOfStacks_.emplace_back(b, prev_stack, pc, stackCount_);

      return true;
    }

    case Opcode::Drop:
      DropKeep(b, 1, 0);
      break;

    case Opcode::InterpDropKeep: {
      uint32_t drop_count = ReadU32(&pc);
      //uint8_t keep_count = *pc++;
      uint32_t keep_count = ReadU32(&pc);
      DropKeep(b, drop_count, keep_count);
      break;
    }

    case Opcode::Nop:
      break;

    default:
      return false;
  }

  int32_t next_index = static_cast<int32_t>(workItems_.size());

  workItems_.emplace_back(OrphanBytecodeBuilder(next_index,
						const_cast<char*>(ReadOpcodeAt(pc).GetName())),
                          pc, stack_, stackCount_);
  b->AddFallThroughBuilder(workItems_[next_index].builder);

  return true;
}

}
}
