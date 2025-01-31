// Copyright 2024 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CT_ICP_ODOMETRY__VISIBILITY_CONTROL_HPP_
#define CT_ICP_ODOMETRY__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(CT_ICP_ODOMETRY_BUILDING_DLL) || defined(CT_ICP_ODOMETRY_EXPORTS)
    #define CT_ICP_ODOMETRY_PUBLIC __declspec(dllexport)
    #define CT_ICP_ODOMETRY_LOCAL
  #else  // defined(CT_ICP_ODOMETRY_BUILDING_DLL) || defined(CT_ICP_ODOMETRY_EXPORTS)
    #define CT_ICP_ODOMETRY_PUBLIC __declspec(dllimport)
    #define CT_ICP_ODOMETRY_LOCAL
  #endif  // defined(CT_ICP_ODOMETRY_BUILDING_DLL) || defined(CT_ICP_ODOMETRY_EXPORTS)
#elif defined(__linux__)
  #define CT_ICP_ODOMETRY_PUBLIC __attribute__((visibility("default")))
  #define CT_ICP_ODOMETRY_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define CT_ICP_ODOMETRY_PUBLIC __attribute__((visibility("default")))
  #define CT_ICP_ODOMETRY_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // CT_ICP_ODOMETRY__VISIBILITY_CONTROL_HPP_
