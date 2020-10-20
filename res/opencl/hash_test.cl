#include "cuckoo.cl"

kernel void testCuckoo(
    const int inputValue,
	global uint* outResult,
	global ulong* cuckoo_table,
	global ulong* cuckoo_stash,
	const uint   cuckoo_prime,
	global uint* cuckoo_hashParams,
	const int    cuckoo_checkStash)
{
	const uint dataIndex = Cuckoo_Find(inputValue,
	    cuckoo_table, cuckoo_stash, cuckoo_prime, cuckoo_hashParams, cuckoo_checkStash);

        const int index = get_global_id(0);
	    outResult[index] = dataIndex;
}