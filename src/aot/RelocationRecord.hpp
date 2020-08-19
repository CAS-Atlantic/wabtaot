#ifndef WASM_RELO_RECORD
#define WASM_RELO_RECORD
#include <time.h>
#include "runtime/OMRRelocationRecord.hpp"
#ifndef WASM_RELOCATION_RECORD_CONNECTOR
#define WASM_RELOCATION_RECORD_CONNECTOR
namespace WASM { class RelocationRecord;}
namespace WASM { typedef WASM::RelocationRecord WASMRelocationRecordConnector;}
#endif
namespace TR { 
  class RelocationRecord; 
  class RelocationRecordWithOffset;
  class RelocationRecordBinaryTemplate;
}

namespace WASM{

class RelocationRecord:public OMR::RelocationRecordConnector{
    public:
        RelocationRecord();
        RelocationRecord(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
        static TR::RelocationRecord* create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime,
					    TR::RelocationTarget *reloTarget, 
					    TR::RelocationRecordBinaryTemplate *record);
};

class RelocationRecordTwo : public WASM::RelocationRecord{
    public:
        RelocationRecordTwo();
        RelocationRecordTwo(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
        virtual void initialize(int dataOne,int dataTwo);
};
/*
struct RelocationRecordWithOffsetPrivateData
   {
      uintptrj_t offset;
   };

union RelocationRecordPrivateData
   {
     WASM::RelocationRecordWithOffsetPrivateData offset;
   };
*/
/*
class RelocationRecordThree : public WASM::RelocationRecord{
    public:
        RelocationRecordThree(int kind);
};
/*
class RelocationRecordMethodCallAddressBinaryTemplate : public RelocationRecordBinaryTemplate{
    public:
        void setMethodAddress(TR::RelocationTarget*, UDATA address);
	UDATA _methodAddress;
};*/

class RelocationRecordWithOffset:public WASM::WASMRelocationRecordConnector{
    public:
        RelocationRecordWithOffset(){}
        RelocationRecordWithOffset(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
        /*static TR::RelocationRecord* create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime, 
					    TR::RelocationTarget *reloTarget, 
					    TR::RelocationRecordBinaryTemplate *record);*/
        uintptr_t offset(TR::RelocationTarget *reloTarget);
	void setOffset(TR::RelocationTarget *reloTarget, UDATA offset); 
};

class RelocationRecordWithOffsetBinaryTemplate : public OMR::RelocationRecordBinaryTemplateConnector{
     public:
         //void setOffset(TR::RelocationTarget*, UDATA offset);
	 UDATA _offset;
};

class RelocationRecordMethodCallAddress : public WASM::RelocationRecord{
    public:
        RelocationRecordMethodCallAddress();
        RelocationRecordMethodCallAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
        virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation);
        virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationHigh, uint8_t *reloLocationLow);
        virtual void preparePrivateData(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget);
        virtual int32_t bytesInHeader(TR::RelocationTarget* a) { return sizeof(RelocationRecordWithOffsetBinaryTemplate); }
        uint8_t *computeTargetMethodAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *baseLocation);
        uint8_t* address(TR::RelocationTarget *reloTarget);
        void setAddress(TR::RelocationTarget *reloTarget, uint8_t* callTargetAddress);
};

class RelocationRecordDataAddress : public WASM::RelocationRecordWithOffset{
     public:
        RelocationRecordDataAddress();
        RelocationRecordDataAddress(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
        virtual int32_t bytesInHeader(TR::RelocationTarget* a) { return sizeof(RelocationRecordWithOffsetBinaryTemplate);}
        //virtual void preparePrivateData(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget);
        virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation);
        virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationHigh, uint8_t *reloLocationLow);
};

class RelocationRecordBranchTable : public RelocationRecordWithOffset
   {
   public:
     RelocationRecordBranchTable() {}
     RelocationRecordBranchTable(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record) : RelocationRecordWithOffset(reloRuntime, record) {}
     virtual int32_t bytesInHeader() { return sizeof(RelocationRecordWithOffsetBinaryTemplate); }
     virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocation);
     virtual int32_t applyRelocation(TR::RelocationRuntime *reloRuntime, TR::RelocationTarget *reloTarget, uint8_t *reloLocationHigh, uint8_t *reloLocationLow);
    };

}

namespace TR{
class RelocationRecord:public WASM::WASMRelocationRecordConnector{
    public:
        RelocationRecord();
        RelocationRecord(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record);
  /*using WASM::WASMRelocationRecordConnector::create;*/
       /* typedef WASM::WASMRelocationRecordConnector::create TR::RelocationRecord::create;*/
       static TR::RelocationRecord* create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime, 
					    TR::RelocationTarget *reloTarget, 
					    TR::RelocationRecordBinaryTemplate *record);
};
class RelocationRecordWithOffset:public WASM::RelocationRecordWithOffset{
    public:
        RelocationRecordWithOffset(){}
        RelocationRecordWithOffset(TR::RelocationRuntime *reloRuntime, TR::RelocationRecordBinaryTemplate *record){}
        /*static TR::RelocationRecord* create(TR::RelocationRecord *storage, TR::RelocationRuntime *reloRuntime, 
					    TR::RelocationTarget *reloTarget, 
					    TR::RelocationRecordBinaryTemplate *record);*/
	void setOffset(TR::RelocationTarget *reloTarget, UDATA offset);
};
class RelocationRecordBinaryTemplate: public OMR::RelocationRecordBinaryTemplateConnector{
    public:
       RelocationRecordBinaryTemplate(){};
};

}
#endif
