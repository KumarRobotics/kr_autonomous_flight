find_path(INTEL_MKL_INCLUDES
  NAMES mkl.h
  PATHS "$ENV{MKLROOT}/include"
  )

find_library(INTEL_LP
  NAMES mkl_intel_lp64
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )
find_library(INTEL_TBB
  NAMES mkl_sequential 
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )
find_library(INTEL_CORE
  NAMES mkl_core
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )



set(INTEL_MKL_LIBRARIES "${INTEL_LP};${INTEL_TBB};${INTEL_CORE}")

# use c++ headers as default
# set(GUROBI_COMPILER_FLAGS "-DIL_STD" CACHE STRING "Gurobi Compiler Flags")

# handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(INTELMKL_DEFAULT_MSG
  INTEL_ILP INTEL_TBB INTEL_CORE INTEL_MKL_INCLUDES)
mark_as_advanced(INTEL_LP INTEL_TBB INTEL_CORE INTEL_MKL_INCLUDES)
