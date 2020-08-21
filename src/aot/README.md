# Wabtaot - WebAssembly AOT compiler

This directory contains the implementation of the WebAssembly Ahead-of-Time compiler.

## Introduction

This AOT compiler is implemented using the Eclipse OMR compiler, relocation and shared code cache modules.
Eclipse OMR code is included in the project as a submodule in the `third_party` directory.

## Building

When the project is compiled, a `wabtaot` executable will be created to represent the AOT compiler and runtime.
Usually built with `make gcc-debug`.

## Code information

Ahead-of-Time compiler functionality is in the `wabt::aot` namespace.

`aot-cd.cc` contains the main function and the code for controlling the parsing of WebAssembly files,
compilation, relocation and executing the `_start` function or exported functions. This file includes
the implementation of WASI or Emscripten interface functions.

`aot-function-builder.h` contains the definition of the AOTFunctionBuilder class that extends the Eclipse OMR
FunctionBuilder class, AOTManager class that handles broadcasting function names and the FunctionImport class.

`aot-function-builder.cc` contains the implementation of the AOT compiler.

`trap-with.cc` contains the implementation of trap messages.

`RelocationRecord.hpp` contains the definition of relocations used in Wabtaot, including the definition of
TR namespace relocations. The implementation of the relocations is in `relocation.cpp`.
