#include <iostream>
#include "RelocationRecord.hpp"
#include "runtime/AOTRelocationRuntime.hpp"
#include "runtime/TRRelocationTarget.hpp"

WASM::RelocationRecord::RelocationRecord():OMR::RelocationRecordConnector(){

}
WASM::RelocationRecord::RelocationRecord(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record):OMR::RelocationRecordConnector(reloRuntime, record){

}
WASM::RelocationRecordTwo::RelocationRecordTwo():WASM::RelocationRecord(){

}
WASM::RelocationRecordTwo::RelocationRecordTwo(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record):WASM::RelocationRecord(reloRuntime, record){

}

WASM::RelocationRecordWithOffset::RelocationRecordWithOffset(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record):WASM::RelocationRecord(reloRuntime, record){
}

/*
WASM::RelocationRecordThree::RelocationRecordThree(int kind):WASM::RelocationRecord(kind){
    std::cout<<"Hello from WASMRelocationRecordThree, with kind"<<kind<<std::endl;
}*/
TR::RelocationRecord* WASM::RelocationRecord::create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, TR::RelocationRecordBinaryTemplate *record){
    TR::RelocationRecord* reloRecord;
    switch (record->type(reloTarget))
    {
        case 2 : reloRecord =reinterpret_cast<TR::RelocationRecord*>(new (storage) WASM::RelocationRecordTwo());
        break;
        case TR_MethodCallAddress : 
	  reloRecord =reinterpret_cast<TR::RelocationRecord*>( new (storage) WASM::RelocationRecordMethodCallAddress(reloRuntime,record)); 
	  break;
	case TR_DataAddress : 
	  reloRecord = reinterpret_cast<TR::RelocationRecord*>( new (storage) WASM::RelocationRecordDataAddress(reloRuntime,record)); 
	  break;
	case TR_BranchTable : 
	  reloRecord = reinterpret_cast<TR::RelocationRecord*>( new (storage) WASM::RelocationRecordBranchTable(reloRuntime,record)); 
	  break;
        break;
        default:
            std::cout<<"CANNOT CREATE WASM RELOCATION RECORD WITH KIND"<<std::endl;
            reloRecord = nullptr;
    }
    return reloRecord;
}
void WASM::RelocationRecordTwo::initialize(int dataOne,int dataTwo){

}

WASM::RelocationRecordMethodCallAddress::RelocationRecordMethodCallAddress():WASM::RelocationRecord(){

}

WASM::RelocationRecordMethodCallAddress::RelocationRecordMethodCallAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record):WASM::RelocationRecord(reloRuntime, record){

}

void 
WASM::RelocationRecordMethodCallAddress::preparePrivateData(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget){
  TR::RelocationRecordMethodCallPrivateData *reloPrivateData = reinterpret_cast<TR::RelocationRecordMethodCallPrivateData *>(&(privateData()->storage))  ;
   uint8_t *baseLocation = 0;
   uint8_t *callTargetAddress = computeTargetMethodAddress(reloRuntime, reloTarget, baseLocation);
   //reloPrivateData->callTargetOffset = (callTargetAddress - baseLocation);
   reloPrivateData->callTargetOffset = reinterpret_cast<uintptr_t>(callTargetAddress);
}

uint8_t *
WASM::RelocationRecordMethodCallAddress::computeTargetMethodAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *baseLocation)
   {
   uint8_t *callTargetAddress = address(reloTarget);
   char methodName[8]{};
   memcpy(methodName,&callTargetAddress,8);
   callTargetAddress = reinterpret_cast<uint8_t*>(reinterpret_cast<TR::AOTRelocationRuntime *>(reloRuntime)->symbolAddress(methodName));
   if (!callTargetAddress) throw std::runtime_error(std::string("symbol not defined: ")+std::string((const char*)methodName));
   return callTargetAddress;
   }

int32_t WASM::RelocationRecordMethodCallAddress::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation)
   {      
     uint8_t* addressOfTheFunction =(uint8_t*)(reinterpret_cast<TR::RelocationRecordMethodCallPrivateData*>(&reinterpret_cast<TR::RelocationRecordPrivateData*>(privateData())->storage)->callTargetOffset);
      reloTarget->storeAddress( addressOfTheFunction, reloLocation);
      return 0;
   }

int32_t WASM::RelocationRecordMethodCallAddress::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationLow, uint8_t *reloLocationHigh)
   {      
      return -1;
   }

uint8_t*
WASM::RelocationRecordMethodCallAddress::address(TR::RelocationTarget *reloTarget)
   {
   RelocationRecordWithOffsetBinaryTemplate *reloData = (RelocationRecordWithOffsetBinaryTemplate *)_record;
   return reloTarget->loadAddress(reinterpret_cast<uint8_t *>(&reloData->_offset));
   }

void
WASM::RelocationRecordMethodCallAddress::setAddress(TR::RelocationTarget *reloTarget, uint8_t *callTargetAddress)
   {
   RelocationRecordWithOffsetBinaryTemplate *reloData = (RelocationRecordWithOffsetBinaryTemplate *)_record;
   reloTarget->storeAddress(callTargetAddress, reinterpret_cast<uint8_t *>(&reloData->_offset));
   }

uintptr_t
WASM::RelocationRecordWithOffset::offset(TR::RelocationTarget *reloTarget)
   {
   return reloTarget->loadRelocationRecordValue((uintptr_t *) &((RelocationRecordWithOffsetBinaryTemplate *)_record)->_offset);
   }

void WASM::RelocationRecordWithOffset::setOffset(TR::RelocationTarget *reloTarget, UDATA offset) { reloTarget->storePointer(reinterpret_cast<uint8_t *>(offset), reinterpret_cast<uint8_t *>(&reinterpret_cast<WASM::RelocationRecordWithOffsetBinaryTemplate*>(_record)->_offset)); }

TR::RelocationRecord* TR::RelocationRecord::create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, TR::RelocationRecordBinaryTemplate *record){
  return WASM::RelocationRecord::create(storage,reloRuntime,reloTarget,record);
}

WASM::RelocationRecordDataAddress::RelocationRecordDataAddress():WASM::RelocationRecordWithOffset(){

}

WASM::RelocationRecordDataAddress::RelocationRecordDataAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record):WASM::RelocationRecordWithOffset(reloRuntime, record){

}
/*
void 
WASM::RelocationRecordDataAddress::preparePrivateData(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget){
   TR::RelocationRecordMethodCallPrivateData *reloPrivateData = &(privateData()->methodCall)  ;
   uint8_t *baseLocation = 0;
   uint8_t *callTargetAddress = computeTargetMethodAddress(reloRuntime, reloTarget, baseLocation);
   //reloPrivateData->callTargetOffset = (callTargetAddress - baseLocation);
   reloPrivateData->callTargetOffset = callTargetAddress;
}*/

int32_t 
WASM::RelocationRecordDataAddress::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation)
   {
   TR::AOTRelocationRuntime *rr = reinterpret_cast<TR::AOTRelocationRuntime*>(reloRuntime);
   char *name = reinterpret_cast<char *>(&reinterpret_cast<WASM::RelocationRecordWithOffsetBinaryTemplate*>(_record)->_offset);
   reloTarget->storeAddress((uint8_t*)rr->symbolAddress(const_cast<char*>(name)), reloLocation);
   return 0;
   }

int32_t WASM::RelocationRecordDataAddress::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationLow, uint8_t *reloLocationHigh)
   {      
      return -1;
   }

int32_t
WASM::RelocationRecordBranchTable::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation)
   {
   TR::AOTRelocationRuntime *rr = reinterpret_cast<TR::AOTRelocationRuntime*>(reloRuntime);
   reloTarget->storeAddress(reloLocation+(UDATA)offset(reloTarget),reloLocation);
   return 0;
   }

int32_t WASM::RelocationRecordBranchTable::applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationLow, uint8_t *reloLocationHigh)
   {      
      return -1;
   }

void TR::RelocationRecordWithOffset::setOffset(TR::RelocationTarget *reloTarget, UDATA offset) { WASM::RelocationRecordWithOffset::setOffset(reloTarget, offset); }

TR::RelocationRecord::RelocationRecord() {}

