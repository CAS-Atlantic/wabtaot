/*
 * Copyright 2018 wasmjit-omr project participants
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

#ifndef JIT_ENVIRONMENT_HPP
#define JIT_ENVIRONMENT_HPP

#include <vector>

namespace wabt {
namespace jit {

class JitEnvironment {
public:
  std::vector<unsigned int> *offsets = 0;
  JitEnvironment();
  ~JitEnvironment();
private:
  static unsigned short instance_count_;
};

}
}

#endif // JIT_ENVIRONMENT_HPP
