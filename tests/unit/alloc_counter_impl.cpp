// The AllocCounter replacement operator new/delete for the test binaries that link this
// file, kept in its own translation unit so GCC does not inline it into the tests and
// report a false -Wmismatched-new-delete.
#define MSS_ALLOC_COUNTER_IMPL
#include "alloc_counter.hpp"
